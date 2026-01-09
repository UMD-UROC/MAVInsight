# python imports
import time

# ROS2 message imports
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

# MAVInsight imports
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import viz_qos

class Site(GraphMember):

    MARKER_TOPIC: str

    def __init__(self):
        super().__init__()

        if self.has_parameter("marker_topic"):
            self.MARKER_TOPIC = self.get_parameter("marker_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("marker_topic")
            self.MARKER_TOPIC = "/site_marker"

        # geofence
        if self.has_parameter("geofence"):
            geofence = self.get_parameter("geofence").get_parameter_value().double_array_value

        # GTs
        if self.has_parameter("ground_truths"):
            ground_truths = self.get_parameter("ground_truths").get_parameter_value().double_array_value

        # map reference
        if self.has_parameter("map_ref"):
            map_ref = self.get_parameter("map_ref").get_parameter_value().double_array_value

        self.map_lla_ref = NavSatFix(
            header=Header(frame_id=self.PARENT_FRAME),
            latitude=map_ref[0],
            longitude=map_ref[1],
            altitude=map_ref[2]
        )
        self.map_ref_pub = self.create_publisher(NavSatFix, "/map_ref", 1)
        self.marker_pub = self.create_publisher(Marker, self.MARKER_TOPIC, viz_qos)

        if len(geofence) % 2 != 0:
            raise ValueError(
                f"Non-even geofence list length. Ensure all lats and lons are paired.\n" +
                f"length: {len(geofence)}"
            )

        vertices = list(zip(geofence[0::2], geofence[1::2]))
        # close the loop
        if vertices[0] != vertices[-1]:
            vertices.append(vertices[0])
        vertices = [lla_2_enu(self.map_lla_ref, NavSatFix(latitude=lat, longitude=lon)) for lat, lon in vertices]
        self.points = [Point(x=e, y=n, z=u) for e, n, u in vertices]

        self.timer = self.create_timer(3, self.site_foxglove_loiter)
        self.i = 0

    def site_foxglove_loiter(self):
        # TODO make this a one-time publish... maybe
        self.marker_pub.publish(Marker(
            header=Header(frame_id=self.PARENT_FRAME),
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=self.points,
            scale=Vector3(x=0.1, y=0.1, z=1.0),
            color=ColorRGBA(r=253.0/256.0, g=138.0/256.0, a=1.0)
        ))
        self.map_ref_pub.publish(self.map_lla_ref)
