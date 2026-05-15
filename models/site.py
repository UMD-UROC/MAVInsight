# python imports
import time

# ROS2 message imports
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

# MAVInsight imports
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import reliable_qos, viz_qos

class Site(GraphMember):

    GEOFENCE_TOPIC: str
    GT_TOPIC: str
    LOCAL_FRAME: str
    MAP_REF_TOPIC: str
    NAME: str

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Site params...")

        self.LOCAL_FIX = None

        # geofence
        if self.has_parameter("geofence_topic"):
            self.GEOFENCE_TOPIC = self.get_parameter("geofence_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("geofence_topic")
            self.GEOFENCE_TOPIC = "/geofence"

        if self.has_parameter("geofence"):
            self.geofence = self.get_parameter("geofence").get_parameter_value().double_array_value
        else:
            self.geofence = None

        # GTs
        if self.has_parameter("ground_truth_topic"):
            self.GT_TOPIC = self.get_parameter("ground_truth_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("ground_truth_topic")
            self.GT_TOPIC = "/ground_truths"

        if self.has_parameter("ground_truths"):
            self.ground_truths = self.get_parameter("ground_truths").get_parameter_value().double_array_value
        else:
            self.ground_truths = None

        # local fix
        if self.has_parameter("local_fix_topic"):
            local_fix_topic = self.get_parameter("local_fix_topic").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Site Node: {self.DISPLAY_NAME} local fix param not set. Unable to initialize Site node.")

        if self.has_parameter("local_frame"):
            self.LOCAL_FRAME = self.get_parameter("local_frame").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Site Node: {self.DISPLAY_NAME} local frame not set. Unable to initialize Site node.")

        if self.has_parameter("name"):
            self.NAME = self.get_parameter("name").get_parameter_value().string_value
        else:
            self.default_parameter_warning("name")
            self.NAME = "site"

        self.timer = self.create_timer(3, self.site_foxglove_loiter)

        self.create_subscription(NavSatFix, local_fix_topic, self.update_local_fix, viz_qos)

        self.geofence_pub = self.create_publisher(Marker, self.GEOFENCE_TOPIC, reliable_qos)
        self.gt_pub = self.create_publisher(Marker, f"/{self.NAME}{self.GT_TOPIC}", reliable_qos)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Site Initialized...")

    def update_local_fix(self, msg: NavSatFix):
        self.LOCAL_FIX = msg

    def site_foxglove_loiter(self):
        if not self.LOCAL_FIX:
            return

        if self.geofence:
            if len(self.geofence) % 2 != 0:
                raise ValueError(
                    f"Non-even geofence list length. Ensure all lats and lons are paired.\n" +
                    f"length: {len(self.geofence)}"
                )
            vertices = list(zip(self.geofence[0::2], self.geofence[1::2]))
            # close the loop
            if vertices[0] != vertices[-1]:
                vertices.append(vertices[0])
            vertices = [lla_2_enu(self.LOCAL_FIX, NavSatFix(latitude=lat, longitude=lon)) for lat, lon in vertices]
            geofence_points = [Point(x=e, y=n, z=u) for e, n, u in vertices]
            self.geofence_pub.publish(Marker(
                header=Header(frame_id=self.LOCAL_FRAME),
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                points=geofence_points,
                scale=Vector3(x=0.1, y=0.1, z=1.0),
                color=ColorRGBA(r=253.0/256.0, g=138.0/256.0, a=1.0)
            ))

        if self.ground_truths:
            gt_coords = list(zip(self.ground_truths[0::2], self.ground_truths[1::2]))
            gt_fixes = [lla_2_enu(self.LOCAL_FIX, NavSatFix(latitude=lat, longitude=lon)) for lat, lon in gt_coords]
            gt_msg_points = [Point(x=e, y=n, z=u) for e, n, u in gt_fixes]

            self.gt_pub.publish(Marker(
                header=Header(frame_id=self.LOCAL_FRAME),
                type=Marker.POINTS,
                action=Marker.ADD,
                points=gt_msg_points,
                scale=Vector3(x=1.0, y=1.0, z=1.0),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            ))
