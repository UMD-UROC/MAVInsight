# python imports
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional


# ROS2 message imports
import mavros_msgs.msg
import px4_msgs.msg
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Header

# ROS imports
from std_msgs.msg import Header

# MAVInsight imports
from models.frame_utils import frd_2_flu, frd_ned_2_flu_enu
from models.frame_member import FrameMember
from models.qos_profiles import viz_qos
from models.sensor_types import SensorTypes

FLAGS_RETRACT = 1
FLAGS_NEUTRAL = 2
FLAGS_ROLL_LOCK = 4
FLAGS_PITCH_LOCK = 8
FLAGS_YAW_LOCK = 16

class Sensor(FrameMember):
    """Class/Node that defines a sensor and its relation to its parent frame. This class
    defines how and what information will be published to Foxglove for every Sensor (i.e.
    TF Frame for position w.r.t. the Vehicle, `Marker`s representing sensor readings, etc)

    Parameters
    ----------
    offset : list[float]
        An [x, y, z] list of values that represents the static offset between the parent
        frame and this frame in meters.
    sensor_type : str
        The type of this Sensor (informed by MAVInsight.models.sensor_types.py enum).
    sensors : list[Sensor]
        A list of other Sensors that are attached to this Sensor. A common example is a
        Camera (Sensor) on a Gimbal (Sensor).
    """

    OFFSET: list[float]
    SENSOR_TYPE: SensorTypes
    SENSORS: list[str]

    # constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Sensor params...")

        # ingest ROS parameters
        # notify user when defaults are being used

        if self.has_parameter("offset"):
            offset_param_val = self.get_parameter("offset").get_parameter_value().double_array_value
            try:
                self.OFFSET = [float(f) for f in offset_param_val]
            except ValueError as e:
                self.OFFSET = []
                self.get_logger().error(
                    f"Unable to interpret offset param elements as floats. Using no-offset default.\n" +
                    f"Received: {offset_param_val}\n" +
                    f"Error: {e}"
                )

            if len(self.OFFSET) != 3:
                self.OFFSET = []
                self.get_logger().error(
                    f"Offset param must be exactly 3 elements long. Using no-offset default.\n" +
                    f"Received: {offset_param_val}"
                )
            else:
                if sum(self.OFFSET) == 0.0:
                    self.OFFSET = []
        else:
            self.default_parameter_warning("offset")
            self.OFFSET = []

        if self.has_parameter("sensor_type"):
            self.SENSOR_TYPE = SensorTypes(self.get_parameter("sensor_type").get_parameter_value().string_value)
        else:
            self.default_parameter_warning("sensor_type")
            self.SENSOR_TYPE = SensorTypes.DEFAULT

        if self.has_parameter("sensors"):
            self.SENSORS = list(self.get_parameter("sensors").get_parameter_value().string_array_value)
        else:
            self.SENSORS = []

        # broadcast the static transform of an offset, if one is present
        if len(self.OFFSET) == 3:
            static_frame_name = f"{self.FRAME_NAME}_offset"
            self.get_logger().info(f"Received valid [x,y,z] sensor offset: {self.OFFSET}m. Building new static TF with child frame: {static_frame_name}")

            # header
            head_out = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.PARENT_FRAME)

            # transform
            # assumed no static rotational offset, for now. TODO
            pos_out = Vector3(x=self.OFFSET[0], y=self.OFFSET[1], z=self.OFFSET[2])
            tf_out = Transform(translation=pos_out)
            static_frame_name = f"{self.FRAME_NAME}_mount"

            # build tf
            s_t = TransformStamped(header=head_out, child_frame_id=static_frame_name, transform=tf_out)
            self.get_logger().debug(f"broadcasting {self.PARENT_FRAME} to {static_frame_name}:\n{s_t}")
            self.tf_static_broadcaster.sendTransform(s_t)

            # allow sub-members to attach to this new offset frame
            self.PARENT_FRAME = static_frame_name

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Sensor initialized!")

    def _format(self, tab_depth: int = 0, extra_fields: str = "") -> str:
        t1 = self._tab_char * tab_depth
        t2 = t1 + self._tab_char
        sensors_string = "[]" if len(self.SENSORS) == 0 else "\n"
        return (
            f"{t1}{self.DISPLAY_NAME} | Sensor {self.SENSOR_TYPE.name}\n" +
            f"{t2}Transform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n" +
            f"{t2}Static offset from parent: (x: {self.OFFSET[0]}, y: {self.OFFSET[1]}, z: {self.OFFSET[2]})\n" +
            extra_fields +
            f"{t2}Sensors: {sensors_string}" +
            "\n".join(t2 + self._tab_char + s for s in self.SENSORS)
        )

    def __str__(self):
        return self._format()

class Camera(Sensor):
    """Class/Node defining a Camera and how/what visualization info will be published for
    Foxglove.

    Parameters
    ----------
    cam_info_topic : str
        The str name of the topic carrying the camera info.
    """

    CAM_INFO_TOPIC: Optional[str]

    # constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Camera params...")

        # ingest ROS parameters
        # notify user when defaults are being used
        if self.has_parameter("cam_info_topic"):
            self.CAM_INFO_TOPIC = self.get_parameter("cam_info_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("cam_info_topic")
            self.CAM_INFO_TOPIC = "camera_info"

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Camera initialized!")

    def _format(self, tab_depth: int = 0, extra_fields: str = "") -> str:
        t = self._tab_char * (tab_depth + 1)
        camera_fields = f"{t}Camera info topic: {self.CAM_INFO_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=camera_fields)

    def __str__(self):
        return self._format()

class Gimbal(Sensor):
    """Class/Node defining a Gimbal and how/what visualization info will be published for
    Foxglove.

    Parameters
    ----------
    orientation_topic : str
        The str name of the topic carrying the gimbal orientation data.
    """

    ORIENTATION_TOPIC: str

    body_orientation: Quaternion

    retract_commanded: bool
    neutral_position_commanded: bool
    roll_lock_commanded: bool
    pitch_lock_commanded: bool
    yaw_lock_commanded: bool

    # constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Camera params...")

        # initialize gimbal state variables
        self.retract_commanded = False
        self.neutral_position_commanded = False
        self.roll_lock_commanded = False
        self.pitch_lock_commanded = False
        self.yaw_lock_commanded = False

        # initialize common gimbal variables
        self.GIMBAL_REF_FRAME_NAME = f"{self.FRAME_NAME}_ref"

        # ingest ROS parameters
        # notify user when defaults are being used
        if self.has_parameter("msg_schema"):
            self.msg_schema = self.get_parameter("msg_schema").get_parameter_value().string_value.lower()
        else:
            self.default_parameter_warning("msg_schema")
            self.msg_schema = "mavros"
        if self.has_parameter("orientation_topic"):
            self.ORIENTATION_TOPIC = self.get_parameter("orientation_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("orientation_topic")
            self.ORIENTATION_TOPIC = "gimbal_orientation" # TODO: Decide on sensible defaults for the position and orientation topic names
        if self.has_parameter("command_topic"):
            self.COMMAND_TOPIC = self.get_parameter("command_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("command_topic")
            self.COMMAND_TOPIC = "command_topic"

        # body orientation topic TODO: Rename. Gimbal may not always be mounted to body...
        if self.has_parameter("body_orientation_topic"):
            self.BODY_TOPIC = self.get_parameter("body_orientation_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("body_orientation_topic")
            self.BODY_TOPIC = "body_topic"

        # initialize subscribers
        match self.msg_schema:
            case "px4_msgs":
                attitude_msg_type = px4_msgs.msg.GimbalDeviceAttitudeStatus
                body_msg_type = px4_msgs.msg.VehicleOdometry
            case "mavros":
                attitude_msg_type = mavros_msgs.msg.GimbalDeviceAttitudeStatus
                body_msg_type = Odometry
                # only initialize subscriber for attitude command messages for mavros. no px4 message currently supported
                self.create_subscription(mavros_msgs.msg.GimbalManagerSetAttitude, self.COMMAND_TOPIC, self.update_commanded_state, viz_qos)
            case _:
                raise ValueError(f"Cannot initialize {self.DISPLAY_NAME} Gimbal viz with message schema: {self.msg_schema}.")
        self.create_subscription(attitude_msg_type, self.ORIENTATION_TOPIC, self.publish_orientation, viz_qos)
        self.create_subscription(body_msg_type, self.BODY_TOPIC, self.update_body, viz_qos)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Gimbal initialized!")

    def update_body(self, msg : Odometry | px4_msgs.msg.VehicleOdometry):
        match self.msg_schema:
            case "px4_msgs":
                assert isinstance(msg, px4_msgs.msg.VehicleOdometry)
                self.body_orientation = frd_ned_2_flu_enu(Quaternion(x=msg.q[1], y=msg.q[2], z=msg.q[3], w=msg.q[0])) #type: ignore
            case "mavros":
                assert isinstance(msg, Odometry)
                self.body_orientation = msg.pose.pose.orientation
            case _:
                raise ValueError(f"Unable to assess body orientation for {self.DISPLAY_NAME} with schema {self.msg_schema}.")

        # re-compute the gimbal's ref frame TODO: Figure out where the following functionality should live.
        R_body_ref = R.identity()

        if self.roll_lock_commanded or self.pitch_lock_commanded or self.yaw_lock_commanded:
            q = self.body_orientation
            R_world_body = R.from_quat([q.x, q.y, q.z, q.w])
            R_body_ref *= R_world_body.inv()

            if not self.yaw_lock_commanded:
                R_body_ref *= self.heading_only_frame()

        (q_x_ref, q_y_ref, q_z_ref, q_w_ref) = R_body_ref.as_quat()
        q_body_ref = Quaternion(x=q_x_ref, y=q_y_ref, z=q_z_ref, w=q_w_ref)

        # publish gimbal ref frame
        tf = TransformStamped(
            header=Header(frame_id=self.PARENT_FRAME),
            child_frame_id=self.GIMBAL_REF_FRAME_NAME,
            transform=Transform(rotation=q_body_ref)
        )
        self.tf_broadcaster.sendTransform(tf)

        # create native coordinate frame if present TODO revisit after establishing basic frame
        # self.get_logger().info(f"Sensor Input Coordinate Frame: {self.COORD_FRAME_TF}")
        # if self.COORD_FRAME_TF == 'enu-flu':
        #     pass
        # elif (self.COORD_FRAME_TF in ['ned-frd']):
        #     native_frame_name = f"{self.FRAME_NAME}_{self.COORD_FRAME_TF}"
        #     self.get_logger().info(f"Recognized non-enu/flu coord frame. Building new static tf endpoint with child frame: {native_frame_name}")
        #     if self.COORD_FRAME_TF == 'ned-frd':

        #         header = Header(frame_id=self.PARENT_FRAME)

        #         (x, y, z, w) = R_frd_flu
        #         ros_quat = Quaternion(x=x, y=y, z=z, w=w)

        #         transform = Transform(rotation=ros_quat)

        #         native_frame=TransformStamped(
        #             header=header,
        #             child_frame_id=native_frame_name,
        #             transform=transform
        #         )
        #     self.tf_static_broadcaster.sendTransform(native_frame)
        # else:
        #     self.get_logger().warn(f"Unrecognized coordinate frame: {self.COORD_FRAME_TF}. Skipping Native Frame creation.")

    def publish_orientation(self, msg : mavros_msgs.msg.GimbalDeviceAttitudeStatus):
        # NOTE: in mavros, GimbalDeviceAttitudeStatus message does NOT reflect commanded flags, only available flags.
        # enu -> d4_base_link -> gimbal_mount -> gimbal_ref_frame -> gimbal_frame

        # construct gimbal attitude frame
        R_ref_g_FRD = R.from_quat([msg.q.x, msg.q.y, msg.q.z, msg.q.w])
        R_ref_g = frd_2_flu(R_ref_g_FRD)
        (g_x, g_y, g_z, g_w) = R_ref_g.as_quat() # type: ignore
        q_ref_g_FLU = Quaternion(x=g_x, y=g_y, z=g_z, w=g_w)

        # publish gimbal orientation tf
        gimbal_tf = TransformStamped(
            header = Header(stamp=msg.header.stamp, frame_id=self.GIMBAL_REF_FRAME_NAME),
            child_frame_id = f"{self.FRAME_NAME}",
            transform = Transform(rotation=q_ref_g_FLU)
        )
        self.tf_broadcaster.sendTransform(gimbal_tf)

    def update_commanded_state(self, msg: mavros_msgs.msg.GimbalManagerSetAttitude):
        # TODO: Change this to a marker. I don't think we need a whole frame for the commanded attitude.
        # Ingest flags
        flags = int(msg.flags)
        self.retract_commanded = bool(flags & FLAGS_RETRACT)
        self.neutral_position_commanded = bool(flags & FLAGS_NEUTRAL)
        self.roll_lock_commanded = bool(flags & FLAGS_ROLL_LOCK)
        self.pitch_lock_commanded = bool(flags & FLAGS_PITCH_LOCK)
        self.yaw_lock_commanded = bool(flags & FLAGS_YAW_LOCK)

        # publish commanded attitude.
        cmd_tf = TransformStamped(
            child_frame_id=f"{self.FRAME_NAME}_commanded_attitude",
            header = Header(frame_id=self.GIMBAL_REF_FRAME_NAME),
            transform = Transform(rotation=frd_2_flu(msg.q))
        )

        self.tf_broadcaster.sendTransform(cmd_tf)

    def heading_only_frame(self) -> R:
        R_world_body = R.from_quat([self.body_orientation.x, self.body_orientation.y, self.body_orientation.z, self.body_orientation.w])
        # find the +x axis of the body (apply the +x vector to the R_world_body frame)
        heading_vector_enu = R_world_body.apply([1.0, 0.0, 0.0])
        # get only the component of this vector in the XY world plane (remove the Z-component of a vector in ENU space)
        heading_vector_enu[2] = 0.0
        # check if the vehicle is pointing straight up or down (would have no measurable "heading")
        norm = np.linalg.norm(heading_vector_enu)
        if norm < 1e-9:
            return R.identity()
        heading_vector_enu = heading_vector_enu / norm
        heading = np.arctan2(heading_vector_enu[1], heading_vector_enu[0])
        return R.from_euler('Z', heading)

    def _format(self, tab_depth: int = 0, extra_fields: str = "") -> str:
        t = self._tab_char * (tab_depth + 1)
        gimbal_fields = f"{t}Orientation topic: {self.ORIENTATION_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=gimbal_fields)

    def __str__(self):
        return self._format()

class Rangefinder(Sensor):
    """Class/Node defining a Rangefinder and how/what visualization info will be published
    for Foxglove.

    Parameters
    ----------
    range_topic : str
        The name of the topic carrying the range info.
    """

    RANGE_TOPIC: Optional[str]

    # constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Rangefinder params...")

        # ingest ROS parameters
        # notify user when defaults are being used.
        if self.has_parameter("range_topic"):
            self.RANGE_TOPIC = self.get_parameter("range_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("range_topic")
            self.RANGE_TOPIC = "rangefinder"

        # initialize subscribers
        self.create_subscription(Range, self.RANGE_TOPIC, self.publish_rangefinder, viz_qos)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Rangefinder initialized!")

    def publish_rangefinder(self, msg: Range):
        d = float(msg.range)

        tf = TransformStamped(
            header = Header(stamp=msg.header.stamp, frame_id=self.PARENT_FRAME),
            child_frame_id = f"{self.FRAME_NAME}",
            transform = Transform(translation=Vector3(x=d))
        )

        self.tf_broadcaster.sendTransform(tf)

    def _format(self, tab_depth: int = 0, extra_fields: str = "") -> str:
        t = self._tab_char * (tab_depth + 1)
        rangefinder_fields = f"{t}Range topic: {self.RANGE_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=rangefinder_fields)

    def __str__(self):
        return self._format()
