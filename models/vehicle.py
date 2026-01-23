# python imports
from __future__ import annotations

# ROS2 message imports
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Transform, TransformStamped, TwistStamped, Vector3
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleOdometry  # type:ignore
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

# MAVInsight imports
from models.frame_utils import enu_2_lla, frd_ned_2_flu_enu
from models.frame_member import FrameMember
from models.platforms import Platforms
from models.qos_profiles import reliable_qos, viz_qos

class Vehicle(FrameMember):
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
        self.latest_header = None
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Vehicle params...")

        # ingest ROS parameters. Notify user when defaults are being used

        # Global Refresh Rate
        if self.has_parameter("refresh_rate"):
            self.REFRESH_RATE = self.get_parameter("refresh_rate").get_parameter_value().double_value
        else:
            self.default_parameter_warning("refresh_rate")
            self.REFRESH_RATE = 60.0  # Hz

        # Namespace
        if self.has_parameter("namespace"):
            namespace = self.get_parameter("namespace").get_parameter_value().string_value
        else:
            self.default_parameter_warning("namespace")
            namespace = "/uas/"

        # ekf origin
        if self.has_parameter("ekf_origin_fix_topic"):
            ekf_topic = self.get_parameter("ekf_origin_fix_topic").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Vehicle Node: {self.DISPLAY_NAME} ekf origin fix topic param not set. Unable to initialize Vehicle node.")

        if self.has_parameter("ekf_origin_frame"):
            self.EKF_FRAME = self.get_parameter("ekf_origin_frame").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Vehicle Node: {self.DISPLAY_NAME} ekf origin frame param not set. Unable to initialize Vehicle node.")

        # Home Position
        if self.has_parameter("home_position_topic"):
            home_pos_topic = self.get_parameter("home_position_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("home_position_topic")
            home_pos_topic = "/home_position/home"

        if self.has_parameter("home_fix_topic"):
            home_fix_topic = self.get_parameter("home_fix_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("home_fix_topic")
            home_fix_topic = "/home_position/fix"

        if self.has_parameter("home_frame_topic"):
            self.HOME_FRAME = self.get_parameter("home_frame_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("home_frame_topic")
            self.HOME_FRAME = "home_position"

        # Location Topic
        if self.has_parameter("location_topic"):
            self.LOCATION_TOPIC = self.get_parameter("location_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("location_topic")
            self.LOCATION_TOPIC = "gps"

        if self.has_parameter("velocity_topic"):
            velocity_topic = self.get_parameter("velocity_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("velocity_topic")
            velocity_topic = "vel"

        # Platform Type
        if self.has_parameter("platform"):
            self.PLATFORM = Platforms(self.get_parameter("platform").get_parameter_value().string_value)
        else:
            self.default_parameter_warning("platform")
            self.PLATFORM = Platforms.DEFAULT

        # Sensors
        if self.has_parameter("sensors"):
            self.SENSORS = list(self.get_parameter("sensors").get_parameter_value().string_array_value)
        else:
            self.SENSORS = []

        # Message Schema
        if self.has_parameter("message_schema"):
            msg_schema_str = self.get_parameter("message_schema").get_parameter_value().string_value
            if msg_schema_str.lower() == "px4_msgs":
                self.LOCATION_MSG_TYPE = VehicleOdometry
            else:
                self.LOCATION_MSG_TYPE = PoseStamped
        else:
            self.default_parameter_warning("message_schema")
            self.LOCATION_MSG_TYPE = PoseStamped

        # Initialize subscribers
        self.create_subscription(self.LOCATION_MSG_TYPE, self.LOCATION_TOPIC, self.publish_position, viz_qos)
        self.create_subscription(HomePosition, home_pos_topic, self.home_cb, viz_qos)
        self.create_subscription(TwistStamped, velocity_topic, self.update_velocity, viz_qos)
        self.VELOCITY = None

        # Initialize publishers
        self.path_pub = self.create_publisher(Path, f"{namespace}flightPath", reliable_qos)
        self.home_fix_pub = self.create_publisher(NavSatFix, home_fix_topic, reliable_qos)
        self.ekf_fix_pub = self.create_publisher(NavSatFix, ekf_topic, reliable_qos)

        # Publisher for velocity vector visualization markers
        self.velocity_vector_marker_pub = self.create_publisher(
            Marker, f"{namespace}velocityVector", reliable_qos
        )

        # Internal storage for path visualizer
        self.path = Path()
        self.path.header.frame_id = self.PARENT_FRAME
        self.latest_pose = None

        # Initialize state variables for velocity and position tracking
        self.drone_velocity = [0.0, 0.0, 0.0]  # Current velocity (m/s)
        self.drone_pos = [0.0, 0.0, 0.0]  # Current position (m)
        self.target_velocity = [0.0, 0.0, 0.0]  # Target velocity (m/s)
        self.target_pos = [0.0, 0.0, 0.0]  # Target position (m)

        # Publisher timers
        self.create_timer(1.0 / self.REFRESH_RATE, self.publish_path)
        self.create_timer(1.0 / self.REFRESH_RATE, self.publish_velocity_vector)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Vehicle initialized!")

    def update_velocity(self, msg: TwistStamped):
        self.VELOCITY=msg

    def publish_position(self, msg: PoseStamped | VehicleOdometry):
        # header
        # TODO: double check time sync between message schemas
        head_out = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.PARENT_FRAME)

        # keep the most recent header for downstream publishers
        self.latest_header = head_out

        path_update = PoseStamped()
        path_update.header = head_out

        # transform
        if self.LOCATION_MSG_TYPE == VehicleOdometry:
            assert isinstance(msg, VehicleOdometry)
            pos_in = msg.position

            pos_out = self.position_conversion(x_in=float(pos_in[0]), y_in=float(pos_in[1]), z_in=float(pos_in[2]))
            q_out_frd = Quaternion(x=float(msg.q[1]), y=float(msg.q[2]), z=float(msg.q[3]), w=float(msg.q[0]))

            q_out_flu = frd_ned_2_flu_enu(q_out_frd)
            tf_out = Transform(translation=pos_out, rotation=q_out_flu)

            path_update.pose.position.x = pos_out.x
            path_update.pose.position.y = pos_out.y
            path_update.pose.position.z = pos_out.z
            path_update.pose.orientation = q_out_flu

            # TODO: Migrate to new function
            vel_in = msg.velocity
            vel_out = self.position_conversion(x_in=float(vel_in[0]), y_in=float(vel_in[1]), z_in=float(vel_in[2]))
            self.drone_velocity = [vel_out.x, vel_out.y, vel_out.z]

            self.drone_pos = [pos_out.x, pos_out.y, pos_out.z]

        else:
            assert isinstance(msg, PoseStamped)
            pos_in = msg.pose.position
            pos_out = Vector3(x=pos_in.x, y=pos_in.y, z=pos_in.z)
            tf_out = Transform(translation=pos_out, rotation=msg.pose.orientation)

            path_update.pose.position.x = float(pos_in.x)
            path_update.pose.position.y = float(pos_in.y)
            path_update.pose.position.z = float(pos_in.z)
            path_update.pose.orientation = msg.pose.orientation

            # Extract velocity from Odometry message
            if self.VELOCITY:
                vel_in = self.VELOCITY.twist.linear
                self.drone_velocity = [float(vel_in.x), float(vel_in.y), float(vel_in.z)]

            self.drone_pos = [float(pos_in.x), float(pos_in.y), float(pos_in.z)]

        # build TF
        t = TransformStamped(
            header=head_out, child_frame_id=self.FRAME_NAME, transform=tf_out
        )

        self.tf_broadcaster.sendTransform(t)

        # build PoseStamped for path
        # Path update
        self.path.poses.append(path_update) # type: ignore
        self.path.header.stamp = path_update.header.stamp

    def home_cb(self, msg: HomePosition):
        home_fix = NavSatFix(
            header=Header(frame_id=self.HOME_FRAME),
            latitude=msg.geo.latitude,
            longitude=msg.geo.longitude,
            altitude=msg.geo.altitude
        )
        self.home_fix_pub.publish(home_fix)
        self.tf_broadcaster.sendTransform(TransformStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.HOME_FRAME),
            child_frame_id=self.EKF_FRAME,
            transform=Transform(translation=Vector3(x=-msg.position.x, y=-msg.position.y, z=-msg.position.z))
        ))

        (lat_e, lon_e, alt_e) = enu_2_lla(home_fix, -msg.position.x, -msg.position.y, -msg.position.z)
        self.ekf_fix_pub.publish(NavSatFix(
            header=Header(frame_id=self.EKF_FRAME),
            latitude=lat_e,
            longitude=lon_e,
            altitude=alt_e
        ))


    def publish_path(self):
        if self.path.poses:
            self.path_pub.publish(self.path)

    def publish_velocity_vector(self):
        # Don't publish until we've received at least one position update
        if self.latest_header is None:
            return

        stamp = self.latest_header.stamp

        target_pos = [
            self.drone_pos[0] + self.drone_velocity[0],
            self.drone_pos[1] + self.drone_velocity[1],
            self.drone_pos[2] + self.drone_velocity[2],
        ]

        velocity_vector_marker = Marker()
        velocity_vector_marker.header.stamp = stamp
        velocity_vector_marker.header.frame_id = self.PARENT_FRAME
        velocity_vector_marker.ns = "velocity_vector"
        velocity_vector_marker.id = 0
        velocity_vector_marker.type = Marker.ARROW
        velocity_vector_marker.action = Marker.ADD

        start_point = Point(
            x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2]
        )

        end_point = Point(x=target_pos[0], y=target_pos[1], z=target_pos[2])
        velocity_vector_marker.points = [start_point, end_point]

        velocity_vector_marker.scale.x = 0.1
        velocity_vector_marker.scale.y = 0.2
        velocity_vector_marker.scale.z = 0.2

        velocity_vector_marker.color.r = 1.0
        velocity_vector_marker.color.a = 1.0

        self.velocity_vector_marker_pub.publish(velocity_vector_marker)

    def position_conversion(self, x_in:float, y_in:float, z_in:float) -> Vector3:
        if 'ned' in self.POSE_FRAME:
            return Vector3(x=y_in, y=x_in, z=-z_in)
        elif 'enu' in self.POSE_FRAME:
            return Vector3(x=x_in, y=y_in, z=z_in)
        else:
            raise ValueError(f"Unable to determine the coordinate frame for message type: {self.POSE_FRAME}")

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
