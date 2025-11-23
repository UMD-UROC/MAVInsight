# python imports
from __future__ import annotations

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

# MAVInsight imports
from models.graph_member import GraphMember
from models.platforms import Platforms
from models.qos_profiles import viz_qos


class Vehicle(GraphMember):
    """Class/Node that defines a generic vehicle (typically a drone) and its sensors.
    This Class defines what information should be published for all Vehicles. (i.e.
    TF Frame for position, velocity `Marker`, etc).

    Parameters
    ----------
    location_topic : str
        The ROS topic that this `Vehicle` should look at to get its location data.
    platform : Platforms | str
        The type of this `Vehicle` (informed by `MAVInsight.models.platforms.py` enum).
    sensors : list[Sensor]
        A list of `Sensors` attached to this vehicle.
    """

    LOCATION_TOPIC: str
    PLATFORM: Platforms
    SENSORS: list[str]

    # constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"Ingesting Vehicle params...")

        # ingest ROS parameters. Notify user when defaults are being used

        # Global Refresh Rate
        if self.has_parameter("refresh_rate"):
            self.REFRESH_RATE = (
                self.get_parameter(
                    "refresh_rate").get_parameter_value().double_value
            )
        else:
            self.default_parameter_warning("refresh_rate")
            self.REFRESH_RATE = 60.0  # Hz

        # Namespace
        if self.has_parameter("namespace"):
            namespace = (
                self.get_parameter(
                    "namespace").get_parameter_value().string_value
            )
        else:
            self.default_parameter_warning("namespace")
            namespace = "/uas/"

        # Location Topic
        if self.has_parameter("location_topic"):
            self.LOCATION_TOPIC = (
                self.get_parameter(
                    "location_topic").get_parameter_value().string_value
            )
        else:
            self.default_parameter_warning("location_topic")
            self.LOCATION_TOPIC = "gps"

        # Platform Type
        if self.has_parameter("platform"):
            self.PLATFORM = Platforms(
                self.get_parameter(
                    "platform").get_parameter_value().string_value
            )
        else:
            self.default_parameter_warning("platform")
            self.PLATFORM = Platforms.DEFAULT

        # Sensors
        if self.has_parameter("sensors"):
            self.SENSORS = list(
                self.get_parameter(
                    "sensors").get_parameter_value().string_array_value
            )
        else:
            self.SENSORS = []

        # Initialize subscribers
        self.create_subscription(
            Odometry, self.LOCATION_TOPIC, self.publish_position, viz_qos
        )
        self.create_subscription(
            PoseStamped, self.LOCATION_TOPIC, self.pose_callback, viz_qos
        )

        # Initialize publishers
        self.path_pub = self.create_publisher(
            Path, f"{namespace}flight-path", 1)

        # Internal storage for path visualizer
        self.path = Path()
        self.path.header.frame_id = self.PARENT_FRAME
        self.latest_pose = None

        # Publisher timers
        self.create_timer(1.0 / self.REFRESH_RATE, self.publish_path)

    def publish_position(self, msg: Odometry):
        # header
        head_out = Header(stamp=msg.header.stamp, frame_id=self.PARENT_FRAME)

        # transform
        pos_in = msg.pose.pose.position
        pos_out = Vector3(x=pos_in.x, y=pos_in.y, z=pos_in.z)
        tf_out = Transform(
            translation=pos_out,
            rotation=msg.pose.pose.orientation)

        # build TF
        t = TransformStamped(
            header=head_out, child_frame_id=self.FRAME_NAME, transform=tf_out
        )

        self.tf_broadcaster.sendTransform(t)

    def pose_callback(self, msg: PoseStamped):
        self.path.poses.append(msg)
        self.path.header.stamp = msg.header.stamp

    def publish_path(self):
        if self.path.poses:
            self.path_pub.publish(self.path)

    def _format(self, tab_depth: int = 0) -> str:
        t1 = self._tab_char * tab_depth
        t2 = t1 + self._tab_char
        sensors_string = "[]" if len(self.SENSORS) == 0 else "\n"
        return (
            f"Vehicle Structure ({self.get_name()}):\n"
            + f"{t1}{self.DISPLAY_NAME} | Vehicle ({self.PLATFORM.name})\n"
            + f"{t2}Transform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n"
            + f"{t2}Location Topic: {self.LOCATION_TOPIC}\n"
            + f"{t2}Sensors: {sensors_string}"
            + ("\n".join(t2 + self._tab_char + s for s in self.SENSORS))
        )

    def __str__(self):
        return self._format()
