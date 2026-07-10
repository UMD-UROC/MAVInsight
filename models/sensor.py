# python imports
from __future__ import annotations

import numpy as np
from typing import Optional
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
import mavros_msgs.msg
import px4_msgs.msg
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Header

# ROS imports
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

# MAVInsight imports
from models.frame_member import FrameMember
from models.frame_utils import euler_2_quat, frd_2_flu, frd_ned_2_flu_enu
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

            if len(self.OFFSET) == 3:
                # append empty RPY values
                self.OFFSET.append(0.0)
                self.OFFSET.append(0.0)
                self.OFFSET.append(0.0)
            elif len(self.OFFSET) == 6:
                pass
            elif len(self.OFFSET) == 0:
                # no offset provided (or unparseable above); no static offset frame
                pass
            else:
                self.OFFSET = []
                self.get_logger().error(
                    f"Offset param must be exactly 3 or 6 elements long (x,y,z or x,y,z,r,p,yaw). Using no-offset default.\n" +
                    f"Received: {offset_param_val}"
                )
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
        if len(self.OFFSET) == 6:
            static_frame_name = f"{self.FRAME_NAME}_offset"
            self.get_logger().info(f"Received valid [x,y,z,r,p,yaw] sensor offset: {self.OFFSET}m. Building new static TF with child frame: {static_frame_name}")

            # header
            head_out = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.PARENT_FRAME)

            # transform
            pos_out = Vector3(x=self.OFFSET[0], y=self.OFFSET[1], z=self.OFFSET[2])
            rot_out = euler_2_quat(self.OFFSET[3], self.OFFSET[4], self.OFFSET[5])

            tf_out = Transform(translation=pos_out, rotation=rot_out)

            # build tf
            self.s_t = TransformStamped(header=head_out, child_frame_id=static_frame_name, transform=tf_out)

            # start up timer to periodically publish this static frame for late-joiners
            self.create_timer(10.0, self.broadcast_static)

            # allow sub-members to attach to this new offset frame
            self.PARENT_FRAME = static_frame_name

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Sensor initialized!")

    def broadcast_static(self):
        self.tf_static_broadcaster.sendTransform(self.s_t)

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

        self.body_orientation = Quaternion()

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

        # TF listeners
        '''
        reducing the buffer length from the default 10 seconds. this listener should only
        ever need to lookup recent body transforms corresponding to the current gimbal
        transform. If this fails, then telemetry data is getting delayed, and we have
        MUCH bigger problems lol. It IS possible that tfs could time out under heavy cpu
        load, though... will need testing/experiment
        '''
        self.tf_buffer = Buffer(Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Gimbal initialized!")

    def publish_orientation(self, msg : mavros_msgs.msg.GimbalDeviceAttitudeStatus):
        # NOTE: in mavros, GimbalDeviceAttitudeStatus message does NOT reflect commanded flags, only available flags. (confirmed)
        # enu -> d4_base_link -> gimbal_offset -> gimbal_ref_frame -> gimbal_frame

        # I am making the decision to remove the gimbal flag handling code from the gimbal
        # class in favor of calculating the gimbal's "reference frame" during the vehicle
        # frame calculations. For drones, the body pose tends to be updated at a much
        # higher rate than the gimbal's pose, so the body should be responsible for
        # maintaining any reference frames that later, lower-rate sensors use.

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
            # crazy that this message doesn't include a header or a timestamp
            header = Header(frame_id=self.GIMBAL_REF_FRAME_NAME),
            transform = Transform(rotation=frd_2_flu(msg.q))
        )

        self.tf_broadcaster.sendTransform(cmd_tf)

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
