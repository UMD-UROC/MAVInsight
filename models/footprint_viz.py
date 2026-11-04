"""Draw the camera footprint that umd_uas computes.

`umd_uas/footprint.py` casts the rays and publishes a `cdcl_umd_msgs/CameraFOV`
of WGS84 points. This node only draws it: an outline in the 3D panel and the
same outline on the Map panel.

The map outline is a closed LineString, not a Polygon, on purpose. A line has
no interior to fill, to tint or to take clicks, so the footprint never covers
what it frames.

Subscribes
    <camera_fov_topic>   cdcl_umd_msgs/CameraFOV
    <local_fix_topic>    sensor_msgs/NavSatFix, the WGS84 position of
                         the fiducial root, when drawing in that frame
    <localization_reference_topic> std_msgs/String, canonical anchor and survey
                         state when drawing in a vehicle localization frame

Reference subscriptions are reliable and transient-local, with depth one.
Vehicle publishes changes immediately and refreshes current state at 1 Hz.
The geographic Map outline remains available while the 3D reference is pending.
Publishes
    <footprint_viz_topic>      visualization_msgs/MarkerArray
    <footprint_geojson_topic>  foxglove_msgs/GeoJSON
"""

# python imports
import json

# ROS2 message imports
from builtin_interfaces.msg import Duration
from cdcl_umd_msgs.msg import CameraFOV
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

# MAVInsight imports
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import latched_reliable_qos, reliable_qos
from mavinsight.localization_reference import ReferenceState, reference_topic

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_GEOJSON = True
except ImportError:
    HAVE_GEOJSON = False


# --------------------------------------------------------------- appearance
OUTLINE_WIDTH_M = 0.5
MAP_OUTLINE_WEIGHT = 2
# A little over one publish period of the compute node, so the outline clears
# when the camera stops reporting instead of hanging where it last looked.
OUTLINE_LIFETIME_SEC = 1


def param(node, name, default):
    """A parameter's value, or its default with the base class's warning."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    node.default_parameter_warning(name)
    return default


class FootprintViz(GraphMember):

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting footprint params...")

        for required in ("camera_fov_topic", "localization_frame"):
            if not self.has_parameter(required):
                raise RuntimeError(
                    f"Footprint viz node: {self.DISPLAY_NAME} has no {required}. "
                    f"Unable to initialize footprint visualization.")
        fov_topic = self.get_parameter("camera_fov_topic").value
        self.LOC_FRAME = self.get_parameter("localization_frame").value

        local_fix_topic = param(self, "local_fix_topic", "ekf_origin/fix")
        viz_topic = param(self, "footprint_viz_topic", "/viz/footprint")
        geojson_topic = param(self, "footprint_geojson_topic", "/viz/footprint_geojson")

        # Red, green and blue are 0 to 255 and alpha is 0 to 1, which is what
        # every marker_color_rgba in this package holds and what tba_viz reads.
        # Scaling alpha as well would take every outline to a thirtieth of a
        # percent opacity, which draws nothing.
        r, g, b, a = param(self, "marker_color_rgba", [255.0, 255.0, 255.0, 0.9])
        self.color = ColorRGBA(r=(r / 255.0), g=(g / 255.0), b=(b / 255.0), a=a)

        self.local_fix = None
        self.correction = None
        # The outline arrives as WGS84 measured from the surveyed frame, so
        # coming back into that frame takes the survey off again.
        fiducial_frame = param(self, "fiducial_frame", "fiducial")
        if self.LOC_FRAME == fiducial_frame:
            # The global root has no survey edge. Its geographic anchor is
            # enough; receiving unrelated vehicle transforms serves no purpose.
            self.correction = (0., 0., 0.)
            self.create_subscription(NavSatFix, local_fix_topic, self.local_fix_cb,
                                     latched_reliable_qos)
        else:
            topic = param(self, "localization_reference_topic",
                          reference_topic(local_fix_topic))
            self.create_subscription(String, topic, self.reference_cb,
                                     latched_reliable_qos)
        self.create_subscription(CameraFOV, fov_topic, self.fov_cb, reliable_qos)
        self.marker_pub = self.create_publisher(MarkerArray, viz_topic, reliable_qos)
        self.geojson_pub = None
        if HAVE_GEOJSON:
            self.geojson_pub = self.create_publisher(GeoJSON, geojson_topic, reliable_qos)
        else:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no footprint. "
                "Install ros-$ROS_DISTRO-foxglove-msgs.")

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Footprint visualization initialized!")

    def local_fix_cb(self, msg: NavSatFix) -> None:
        self.local_fix = msg

    def reference_cb(self, msg: String) -> None:
        """Read anchor and correction atomically, including explicit resets."""
        try:
            state = ReferenceState.decode(msg.data)
            lat, lon, alt = state.frame_anchor(self.LOC_FRAME, corrected=False)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warn(
                f"invalid footprint localization reference: {error}",
                throttle_duration_sec=10.0)
            return
        self.local_fix = NavSatFix(latitude=lat, longitude=lon, altitude=alt)
        self.correction = ((0., 0., 0.) if self.LOC_FRAME == state.raw_frame
                           else state.correction)

    def fov_cb(self, msg: CameraFOV) -> None:
        if not msg.fov_polygon:
            return
        self.publish_geojson(msg)
        if self.local_fix is None or self.correction is None:
            self.get_logger().warn(
                "waiting for geographic reference, so the footprint cannot be placed in the 3D panel",
                throttle_duration_sec=10.0)
            return

        # ignore_alt must stay False: the default substitutes the reference's
        # altitude for the point's, which would flatten the outline onto the
        # frame origin's height instead of draping it over the ground.
        correction = self.correction
        points = [Point(x=e - correction[0], y=n - correction[1],
                        z=u - correction[2]) for e, n, u in
                  (lla_2_enu(self.local_fix, fix, ignore_alt=False)
                   for fix in msg.fov_polygon)]
        outline = Marker(
            header=Header(frame_id=self.LOC_FRAME, stamp=msg.header.stamp),
            ns="footprint",
            id=0,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=points + points[:1],
            scale=Vector3(x=OUTLINE_WIDTH_M),
            color=self.color,
            lifetime=Duration(sec=OUTLINE_LIFETIME_SEC),
        )
        outline.pose.orientation.w = 1.0
        self.marker_pub.publish(MarkerArray(markers=[outline]))

    def publish_geojson(self, msg: CameraFOV) -> None:
        """The outline on the Map panel. The Map panel merges `style` over the
        topic color, so leaving the color out keeps this camera's own."""
        if self.geojson_pub is None:
            return
        ring = [[fix.longitude, fix.latitude] for fix in msg.fov_polygon]
        ring.append(ring[0])
        feature = {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": ring},
            "properties": {
                "name": f"{msg.header.frame_id} footprint",
                "metadata": {"coverage_m2": round(float(msg.coverage_area))},
                "style": {"weight": MAP_OUTLINE_WEIGHT},
            },
        }
        self.geojson_pub.publish(GeoJSON(geojson=json.dumps(
            {"type": "FeatureCollection", "features": [feature]})))
