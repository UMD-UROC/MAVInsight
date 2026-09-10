"""Draw the vehicle on the Map panel, pointing where it points.

A `sensor_msgs/NavSatFix` carries no heading, so the Map panel's own vehicle
dot cannot say which way the vehicle faces. `foxglove_msgs/LocationFix`
carries one, and the panel draws an arrowhead that turns with it.

The position and the covariance are the vehicle's own fix, passed through, so
this marker cannot disagree with the fix the panel already shows: the two
schemas share the covariance convention and its constants. The heading goes
out as NaN, which the schema reads as "not set", until the first compass
message.

Subscribes
    <fix_topic>       sensor_msgs/NavSatFix
    <heading_topic>   std_msgs/Float64, degrees
Publishes
    <location_viz_topic>   foxglove_msgs/LocationFix, heading in radians
"""

# python imports
import math

# ROS2 message imports
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# MAVInsight imports
from models.graph_member import GraphMember
from models.qos_profiles import reliable_qos, viz_qos

try:
    from foxglove_msgs.msg import LocationFix
    HAVE_LOCATION_FIX = True
except ImportError:
    HAVE_LOCATION_FIX = False


# --------------------------------------------------------------- appearance
# The fix arrives at whatever rate MAVROS gives it. The panel needs a turning
# arrowhead, not every sample, so this resamples the pair.
PUBLISH_RATE_HZ = 5.0


def param(node, name, default):
    """A parameter's value, or its default with the base class's warning."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    node.default_parameter_warning(name)
    return default


class LocationViz(GraphMember):

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting location params...")

        fix_topic = param(self, "fix_topic", "global_position/global")
        heading_topic = param(self, "heading_topic", "global_position/compass_hdg")
        pose_topic = param(self, "pose_topic", "local_position/pose")
        use_local_pose = bool(param(self, "use_local_pose", False))
        use_tf_pose = bool(param(self, "use_tf_pose", False))
        reference_frame = param(self, "reference_frame", "home_position")
        base_frame = param(self, "base_frame", "base_link")
        relative_altitude = float(param(self, "relative_altitude", float("nan")))
        viz_topic = param(self, "location_viz_topic", "/viz/location")

        self.fix = None
        self.pose = None
        self.use_local_pose = use_local_pose
        self.use_tf_pose = use_tf_pose
        self.reference_frame = reference_frame
        self.base_frame = base_frame
        self.relative_altitude = relative_altitude
        self.tf_buffer = Buffer() if self.use_tf_pose else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.use_tf_pose else None
        self.heading_rad = float("nan")

        if not HAVE_LOCATION_FIX:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no vehicle "
                "heading. Install ros-$ROS_DISTRO-foxglove-msgs.")
            return

        # Best effort on both: MAVROS publishes its telemetry best effort, and
        # a reliable subscription to it matches no publisher and receives
        # nothing at all.
        self.create_subscription(NavSatFix, fix_topic, self.fix_cb, viz_qos)
        self.create_subscription(Float64, heading_topic, self.heading_cb, viz_qos)
        if self.use_local_pose and not self.use_tf_pose:
            self.create_subscription(PoseStamped, pose_topic, self.pose_cb, viz_qos)
        self.location_pub = self.create_publisher(LocationFix, viz_topic, reliable_qos)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_location)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Location visualization initialized!")

    def fix_cb(self, msg: NavSatFix) -> None:
        # A NO_FIX message carries zeros, which would put the vehicle off the
        # coast of Africa and take the panel's view with it.
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.fix = msg

    def heading_cb(self, msg: Float64) -> None:
        """The compass reports degrees. LocationFix carries radians."""
        self.heading_rad = math.radians(float(msg.data))

    def pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _vehicle_fix(self) -> tuple[float, float, float, object]:
        if self.use_tf_pose:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame, self.base_frame, Time())
            except TransformException:
                return None
            translation = transform.transform.translation
            east = translation.x
            north = translation.y
            altitude = self.fix.altitude + translation.z
            q = transform.transform.rotation
            yaw_enu = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.heading_rad = (math.pi / 2.0 - yaw_enu) % (2.0 * math.pi)
            radius = 6378137.0
            latitude = self.fix.latitude + math.degrees(north / radius)
            longitude = self.fix.longitude + math.degrees(
                east / (radius * math.cos(math.radians(self.fix.latitude))))
            return latitude, longitude, altitude, transform.header
        if not self.use_local_pose or self.pose is None:
            return self.fix.latitude, self.fix.longitude, self.fix.altitude, self.fix.header
        east = self.pose.pose.position.x
        north = self.pose.pose.position.y
        radius = 6378137.0
        latitude = self.fix.latitude + math.degrees(north / radius)
        longitude = self.fix.longitude + math.degrees(
            east / (radius * math.cos(math.radians(self.fix.latitude))))
        altitude = (self.fix.altitude + self.relative_altitude
                    if math.isfinite(self.relative_altitude)
                    else self.fix.altitude + self.pose.pose.position.z)
        q = self.pose.pose.orientation
        yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.heading_rad = (math.pi / 2.0 - yaw_enu) % (2.0 * math.pi)
        return latitude, longitude, altitude, self.pose.header

    def publish_location(self) -> None:
        if self.fix is None:
            return
        location = LocationFix()
        vehicle_fix = self._vehicle_fix()
        if vehicle_fix is None:
            return
        latitude, longitude, altitude, header = vehicle_fix
        location.timestamp = header.stamp
        location.frame_id = header.frame_id
        location.latitude = latitude
        location.longitude = longitude
        location.altitude = altitude
        location.position_covariance = self.fix.position_covariance
        location.position_covariance_type = self.fix.position_covariance_type
        location.heading = self.heading_rad
        self.location_pub.publish(location)
