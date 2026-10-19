"""Place corrected Spot GPS fixes directly in the fiducial frame."""

from bisect import bisect_left
import math

import pymap3d as pm
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from visualization_msgs.msg import Marker, MarkerArray

SPOT_NAMES = ('apollo', 'balto', 'cairo')
REJECTION_RADIUS_M = 2.0


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def valid_position(lat, lon):
    return (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90
            and -180 <= lon <= 180 and (lat != 0 or lon != 0))


class SpotFrames:
    """Publish Spot TF and markers. Query the same TF buffer at detection capture times.

    Each edge is fiducial -> <name>_base_link. Received GPS fixes already contain
    ground localization corrections. No UAS home or fiducial correction enters this path.
    Only exact or bracketed TF samples are valid. Both brackets must be recent.
    """
    def __init__(self, node=None, names=SPOT_NAMES, max_age_sec=2.0, cache_sec=3600.0):
        if max_age_sec <= 0 or not math.isfinite(max_age_sec):
            raise ValueError('Spot max_age_sec must be finite and positive')
        self.names = tuple(names)
        self.max_age_ns = int(max_age_sec * 1e9)
        self.limit = 36000
        self.samples = {name: [] for name in names}
        self.pending = {}
        self.anchor = None
        self.buffer = Buffer(cache_time=Duration(seconds=cache_sec))
        self.broadcaster = TransformBroadcaster(node) if node is not None else None
        self.markers = node.create_publisher(MarkerArray, '/viz/spots', 10) if node is not None else None
        self.subscriptions = []
        if node is not None:
            fix_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions.append(node.create_subscription(
                NavSatFix, '/fiducial/fix', self.set_anchor, fix_qos))
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            for name in names:
                self.subscriptions.append(node.create_subscription(
                    NavSatFix, f'/{name}/position', lambda msg, name=name: self.add(name, msg), qos))

    def set_anchor(self, msg):
        anchor = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
        if not valid_position(*anchor[:2]) or not math.isfinite(anchor[2]):
            return
        if self.anchor == anchor:
            return
        self.anchor = anchor
        self.buffer.clear()
        for times in self.samples.values():
            times.clear()
        pending, self.pending = self.pending, {}
        for name, messages in pending.items():
            for message in messages:
                self.add(name, message)

    def add(self, name, msg):
        t = stamp_ns(msg.header.stamp)
        if (name not in self.samples or t <= 0 or msg.status.status < 0
                or not valid_position(msg.latitude, msg.longitude)):
            return
        if self.anchor is None:
            messages = self.pending.setdefault(name, [])
            messages.append(msg)
            del messages[:-self.limit]
            return
        altitude = msg.altitude if math.isfinite(msg.altitude) else self.anchor[2]
        xyz = pm.geodetic2enu(msg.latitude, msg.longitude, altitude, *self.anchor)
        transform = TransformStamped()
        transform.header.frame_id = 'fiducial'
        transform.header.stamp = msg.header.stamp
        transform.child_frame_id = f'{name}_base_link'
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.w = 1.0
        self.buffer.set_transform(transform, 'spot_frames')
        times = self.samples[name]
        i = bisect_left(times, t)
        if i == len(times) or times[i] != t:
            times.insert(i, t)
        del times[:-self.limit]
        if self.broadcaster is not None:
            self.broadcaster.sendTransform(transform)
            self.markers.publish(spot_markers(name, transform))

    def distance_to(self, name, t, lat, lon):
        if self.anchor is None or name not in self.samples:
            return None
        times = self.samples[name]
        i = bisect_left(times, t)
        exact = i < len(times) and times[i] == t
        if not exact and not (0 < i < len(times) and t - times[i - 1] <= self.max_age_ns
                              and times[i] - t <= self.max_age_ns):
            return None
        try:
            # Exact capture time, never Time() (latest). TF interpolates between Spot fixes.
            transform = self.buffer.lookup_transform(
                f'{name}_base_link', 'fiducial', Time(nanoseconds=int(t)))
        except TransformException:
            return None
        # The Spot edge has identity rotation. This inverse TF puts the detection in Spot ENU.
        east, north, _ = pm.geodetic2enu(lat, lon, self.anchor[2], *self.anchor)
        shift = transform.transform.translation
        return math.hypot(east + shift.x, north + shift.y)


def spot_markers(name, transform):
    """A Spot body, a label, and its two-meter horizontal exclusion disk."""
    result = MarkerArray()
    colors = {'apollo': (1.0, 0.65, 0.1), 'balto': (0.1, 0.8, 1.0), 'cairo': (0.8, 0.4, 1.0)}
    color = colors.get(name, (1.0, 0.8, 0.0))
    for index, kind in enumerate((Marker.CUBE, Marker.TEXT_VIEW_FACING, Marker.CYLINDER, Marker.LINE_STRIP)):
        marker = Marker()
        marker.header.frame_id = transform.child_frame_id
        marker.header.stamp = transform.header.stamp
        marker.ns = name
        marker.id = index
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=4.0).to_msg()
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        if kind == Marker.CUBE:
            marker.scale.x, marker.scale.y, marker.scale.z = 0.9, 0.45, 0.35
        elif kind == Marker.TEXT_VIEW_FACING:
            marker.pose.position.z = 0.8
            marker.scale.z = 0.35
            marker.text = f'{name} (Spot) | 2 m'
        elif kind == Marker.CYLINDER:
            marker.scale.x = marker.scale.y = 2 * REJECTION_RADIUS_M
            marker.scale.z = 0.04
            marker.color.a = 0.15
        else:
            marker.scale.x = 0.04
            marker.points = [Point(x=REJECTION_RADIUS_M * math.cos(2 * math.pi * k / 64),
                                   y=REJECTION_RADIUS_M * math.sin(2 * math.pi * k / 64), z=0.03)
                             for k in range(65)]
        result.markers.append(marker)
    return result


def main(args=None):
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=args)
    node = Node('spot_viz')
    node.declare_parameter('spot_names', list(SPOT_NAMES))
    frames = SpotFrames(node, names=node.get_parameter('spot_names').value)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
