# python imports
from __future__ import annotations

from math import isclose
from typing import Optional, Tuple

import numpy as np
from cdcl_umd_msgs.msg import FiducialCalibration
import pymap3d as pm
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Transform, TransformStamped, TwistStamped, Vector3
from mavros_msgs.msg import Altitude, HomePosition, GimbalDeviceAttitudeStatus
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# MAVInsight imports
from models.frame_utils import enu_2_lla, frd_ned_2_flu_enu
from mavinsight.localization_reference import LocalizationReference, ReferenceState, stamp_ns
from models.frame_member import FrameMember
from models.gimbal_frame import (FLAGS_NEUTRAL, FLAGS_PITCH_LOCK,
                                 FLAGS_RETRACT, FLAGS_ROLL_LOCK,
                                 FLAGS_YAW_IN_EARTH_FRAME,
                                 FLAGS_YAW_IN_VEHICLE_FRAME, FLAGS_YAW_LOCK,
                                 gimbal_reference_from_body,
                                 yaw_is_earth_referenced)
from models.platforms import Platforms
from models.qos_profiles import latched_reliable_qos, reliable_qos, viz_qos


class FlightPathChunks:
    def __init__(self, frame_id: str, chunk_size: int):
        self.frame_id = frame_id
        self.chunk_size = chunk_size
        self.chunks: list[list[PoseStamped]] = []
        self.published = 0

    def add(self, pose: PoseStamped):
        if not self.chunks or len(self.chunks[-1]) >= self.chunk_size:
            self.chunks.append([])
        self.chunks[-1].append(pose)

    def message(self) -> MarkerArray:
        output = MarkerArray()
        last = len(self.chunks) - 1
        first = self.published
        for index in range(first, len(self.chunks)):
            chunk = self.chunks[index]
            if index == last or index >= self.published:
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = chunk[-1].header.stamp
                marker.ns = "flight_path"
                marker.id = index
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.15
                marker.color.a = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.points = [Point(x=p.pose.position.x, y=p.pose.position.y,
                                       z=p.pose.position.z) for p in chunk]
                output.markers.append(marker)
        self.published = max(self.published, last)
        return output


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
        if self.has_parameter("altitude_topic"):
            alt_topic = self.get_parameter("altitude_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning('altitude_topic')
            alt_topic = "/altitude"

        # Global Refresh Rate
        if self.has_parameter("refresh_rate"):
            self.REFRESH_RATE = self.get_parameter("refresh_rate").get_parameter_value().double_value
        else:
            self.default_parameter_warning("refresh_rate")
            self.REFRESH_RATE = 60.0  # Hz
        self.PATH_CHUNK_SIZE = int(self.get_parameter("path_chunk_size").value) if self.has_parameter("path_chunk_size") else 256
        self.PATH_PUBLISH_RATE = float(self.get_parameter("path_publish_rate").value) if self.has_parameter("path_publish_rate") else 1.0

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

        if self.has_parameter("fiducial_frame"):
            self.FIDUCIAL_FRAME = self.get_parameter("fiducial_frame").get_parameter_value().string_value
        else:
            self.FIDUCIAL_FRAME = "fiducial"

        if self.has_parameter("fiducial_update_topic"):
            fiducial_update_topic = self.get_parameter("fiducial_update_topic").get_parameter_value().string_value
        else:
            fiducial_update_topic = "fiducial_update"

        self.publish_fiducial_edge = bool(
            self.get_parameter('publish_fiducial_edge').value
            if self.has_parameter('publish_fiducial_edge') else True)

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

        if self.has_parameter("home_frame_name"):
            self.HOME_FRAME = self.get_parameter("home_frame_name").get_parameter_value().string_value
        else:
            self.default_parameter_warning("home_frame_name")
            self.HOME_FRAME = "home_position"

        if self.has_parameter("uncorrected_home_frame_name"):
            self.UNCORRECTED_HOME_FRAME = self.get_parameter(
                "uncorrected_home_frame_name").get_parameter_value().string_value
        else:
            self.default_parameter_warning("uncorrected_home_frame_name")
            suffix = "_home_position"
            self.UNCORRECTED_HOME_FRAME = (
                f"{self.HOME_FRAME[:-len(suffix)]}_home_uncorrected"
                if self.HOME_FRAME.endswith(suffix)
                else f"{self.HOME_FRAME}_uncorrected")

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

        # Position tolerance for path de-duplication. A vehicle sitting still still
        # streams pose at the full rate, and every one of those samples used to become
        # a Path point, so an idle aircraft grew the message without moving. A sample
        # within this radius of the last one that was kept is dropped.
        if self.has_parameter("position_tolerance"):
            self.POSITION_TOLERANCE = float(self.get_parameter("position_tolerance").value)
        else:
            self.default_parameter_warning("position_tolerance")
            self.POSITION_TOLERANCE = 0.0254  # 1 inch in meters
        self.BENCH_BASE_ALTITUDE = float(
            self.get_parameter("bench_base_altitude").value
            if self.has_parameter("bench_base_altitude") else float("nan"))

        # A two-axis mount cannot acquire an earth-fixed yaw, even if PX4
        # briefly reports the preceding mission-ROI lock during handoff.
        # Keep this capability beside the vehicle model so TF consumers do not
        # each infer it from asynchronous gimbal status.
        self.GIMBAL_HAS_YAW_AXIS = bool(
            self.get_parameter("gimbal.has_yaw_axis").value
            if self.has_parameter("gimbal.has_yaw_axis") else True)

        # Message Schema
        if self.has_parameter("message_schema"):
            msg_schema_str = self.get_parameter("message_schema").get_parameter_value().string_value
            self.LOCATION_MSG_TYPE = PoseStamped
        else:
            self.default_parameter_warning("message_schema")
            self.LOCATION_MSG_TYPE = PoseStamped

        # Initialize subscribers
        self.create_subscription(Altitude, alt_topic, self.update_alt, viz_qos)
        self.create_subscription(self.LOCATION_MSG_TYPE, self.LOCATION_TOPIC, self.publish_position, viz_qos)
        self.create_subscription(HomePosition, home_pos_topic, self.home_cb, viz_qos)
        self.create_subscription(TwistStamped, velocity_topic, self.update_velocity, viz_qos)
        self.create_subscription(
            TransformStamped, fiducial_update_topic, self.update_fiducial,
            latched_reliable_qos)
        self.ALTITUDE = None
        calibration_topic = (fiducial_update_topic.rsplit('/', 1)[0] +
                             '/fiducial_calibration/update'
                             if '/' in fiducial_update_topic else
                             'fiducial_calibration/update')
        self.create_subscription(FiducialCalibration, calibration_topic,
                                 self.update_calibration, latched_reliable_qos)
        self.VELOCITY = None

        # Initialize publishers
        self.path_pub = self.create_publisher(MarkerArray, f"{namespace}flightPathChunks", reliable_qos)
        # Home is state, not high-rate telemetry.  Keep the latest value for
        # late-joining ground consumers (fleet_tf in particular).
        self.home_fix_pub = self.create_publisher(NavSatFix, home_fix_topic, latched_reliable_qos)
        self.reference_pub = self.create_publisher(
            String, home_fix_topic.rsplit('/', 2)[0] + '/localization/reference',
            latched_reliable_qos)
        self.reference_events_pub = self.create_publisher(
            String, home_fix_topic.rsplit('/', 2)[0] + '/localization/reference_events',
            reliable_qos)
        self.localization_reference = LocalizationReference()
        self.external_reference_topic = (self.get_parameter('localization_reference_topic').value
                                         if self.has_parameter('localization_reference_topic') else '')
        if self.external_reference_topic:
            self.create_subscription(String, self.external_reference_topic,
                                     self.reference_cb, latched_reliable_qos)
            self.create_subscription(String, self.external_reference_topic.replace(
                '/reference/', '/reference_events/'), self.reference_cb, reliable_qos)
        self.ekf_fix_pub = self.create_publisher(NavSatFix, ekf_topic, reliable_qos)
        self.velocity_vector_pub = self.create_publisher(Marker, f"{namespace}velocityVector", reliable_qos)

        # Internal storage for path visualizer
        self.path = FlightPathChunks(self.PARENT_FRAME, self.PATH_CHUNK_SIZE)
        self.latest_pose = None

        # Initialize state variables for velocity and position tracking
        self.drone_velocity = [0.0, 0.0, 0.0]  # Current velocity (m/s)
        self.drone_pos = [0.0, 0.0, 0.0]  # Current position (m)
        self.last_drone_pos: Optional[Tuple[float, float, float]] = None  # Last position kept on the path
        self.target_velocity = [0.0, 0.0, 0.0]  # Target velocity (m/s)
        self.target_pos = [0.0, 0.0, 0.0]  # Target position (m)

        # The stabilization lock flags come from the gimbal's own report, not
        # from the manager command that asked for them. MAVROS publishes
        # `manager/set_attitude` outbound, so a ground station listening on the
        # same MAVLink stream never sees it, while GIMBAL_DEVICE_ATTITUDE_STATUS
        # already streams and carries the same RETRACT, NEUTRAL, ROLL_LOCK,
        # PITCH_LOCK and YAW_LOCK bits. It also reports what the gimbal did
        # rather than what was requested.

        # Guarded like every other parameter here. A config that omits one of
        # these used to raise in the constructor, which took the whole frame
        # tree down before a single transform went out.
        if self.has_parameter("gimbal_flags_topic"):
            gimbal_flags_topic = self.get_parameter("gimbal_flags_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("gimbal_flags_topic")
            gimbal_flags_topic = "gimbal_control/device/attitude_status"

        if self.has_parameter("gimbal_offset_frame"):
            self.gimbal_offset_frame = self.get_parameter("gimbal_offset_frame").get_parameter_value().string_value
        else:
            self.default_parameter_warning("gimbal_offset_frame")
            self.gimbal_offset_frame = "gimbal_frame_offset"

        if self.has_parameter("gimbal_ref_frame"):
            self.gimbal_ref_frame = self.get_parameter("gimbal_ref_frame").get_parameter_value().string_value
        else:
            self.default_parameter_warning("gimbal_ref_frame")
            self.gimbal_ref_frame = "gimbal_frame_ref"

        if self.has_parameter("gimbal_reference_apply_stabilization_correction"):
            self.gimbal_reference_apply_stabilization_correction = (
                self.get_parameter("gimbal_reference_apply_stabilization_correction")
                .get_parameter_value().bool_value)
        else:
            self.default_parameter_warning("gimbal_reference_apply_stabilization_correction")
            self.gimbal_reference_apply_stabilization_correction = True

        if self.has_parameter("gimbal_reference_yaw_frame"):
            yaw_frame = self.get_parameter("gimbal_reference_yaw_frame").get_parameter_value().string_value
            if yaw_frame not in ("reported", "earth"):
                raise ValueError("gimbal_reference_yaw_frame must be 'reported' or 'earth'")
            self.gimbal_reference_yaw_is_earth = (yaw_frame == "earth")
        else:
            self.default_parameter_warning("gimbal_reference_yaw_frame")
            self.gimbal_reference_yaw_is_earth = None

        if self.has_parameter("gimbal_reference_rotation_deg"):
            rotation_deg = list(
                self.get_parameter("gimbal_reference_rotation_deg")
                .get_parameter_value().double_array_value)
            if len(rotation_deg) != 3:
                raise ValueError(
                    "gimbal_reference_rotation_deg must contain roll, pitch, and yaw in degrees")
        else:
            self.default_parameter_warning("gimbal_reference_rotation_deg")
            rotation_deg = [0.0, 0.0, 0.0]
        self.gimbal_reference_rotation = R.from_euler("xyz", rotation_deg, degrees=True)

        self.create_subscription(GimbalDeviceAttitudeStatus, gimbal_flags_topic, self.update_gimbal_flags, viz_qos)
        # initialize gimbal state variables
        self.retract_commanded = False
        self.neutral_position_commanded = False
        self.roll_lock_commanded = False
        self.pitch_lock_commanded = False
        self.yaw_lock_commanded = False
        self.gimbal_flags = 0

        # Publisher timers
        self.create_timer(1.0 / self.PATH_PUBLISH_RATE, self.publish_path)
        self.create_timer(1.0 / self.REFRESH_RATE, self.publish_velocity_vector)

        # Split global GPS placement from the survey correction so both are
        # inspectable: fiducial -> home_uncorrected -> home_position.
        self.raw_home_t = TransformStamped()
        self.raw_home_t.header = Header(
            frame_id=self.FIDUCIAL_FRAME, stamp=self.get_clock().now().to_msg())
        self.raw_home_t.child_frame_id = self.UNCORRECTED_HOME_FRAME
        self.raw_home_t.transform.rotation.w = 1.0
        self.correction_t = TransformStamped()
        self.correction_t.header = Header(
            frame_id=self.UNCORRECTED_HOME_FRAME,
            stamp=self.get_clock().now().to_msg())
        self.correction_t.child_frame_id = self.HOME_FRAME
        self.correction_t.transform.rotation.w = 1.0
        self._fiducial_correction = Vector3()
        self._fiducial_lla = list(self.get_parameter('fiducial_lla').value) \
            if self.has_parameter('fiducial_lla') else None
        self._fiducial_fix_pub = self.create_publisher(NavSatFix, '/fiducial/fix', reliable_qos)

        # Stable application HOME -> EKF offset. Atomic H-h composition
        # absorbs paired PX4 home changes before any consumer sees them. A static transform has no time
        # extent, so a move rewrites the whole past and every lookup already in
        # flight silently changes answer. It is sent from the pose callback
        # instead of from home_cb, because mavros can go minutes without
        # republishing home_position/home and one sample per update leaves the
        # chain un-lookupable in between.
        self.home_t = None

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Vehicle initialized!")

    def update_gimbal_flags(self, msg: GimbalDeviceAttitudeStatus):
        flags = int(msg.flags)
        new_flag = False
        new_flag |= (yaw_is_earth_referenced(self.gimbal_flags)
                     != yaw_is_earth_referenced(flags))
        new_flag |= (self.retract_commanded ^ bool(flags & FLAGS_RETRACT))
        new_flag |= (self.neutral_position_commanded ^ bool(flags & FLAGS_NEUTRAL))
        new_flag |= (self.roll_lock_commanded ^ bool(flags & FLAGS_ROLL_LOCK))
        new_flag |= (self.pitch_lock_commanded ^ bool(flags & FLAGS_PITCH_LOCK))
        new_flag |= (self.yaw_lock_commanded ^ bool(flags & FLAGS_YAW_LOCK))
        self.retract_commanded = bool(flags & FLAGS_RETRACT)
        self.neutral_position_commanded = bool(flags & FLAGS_NEUTRAL)
        self.roll_lock_commanded = bool(flags & FLAGS_ROLL_LOCK)
        self.pitch_lock_commanded = bool(flags & FLAGS_PITCH_LOCK)
        self.yaw_lock_commanded = bool(flags & FLAGS_YAW_LOCK)
        self.gimbal_flags = flags
        if new_flag:
            yaw_frame = "earth" if yaw_is_earth_referenced(flags) else "vehicle"
            self.get_logger().info(
                f"~~~~~~~NEW Gimbal Flags~~~~~~~~\n"
                f"Retract: {self.retract_commanded}\n"
                f"Neutral: {self.neutral_position_commanded}\n"
                f"Roll Lock: {self.roll_lock_commanded}\n"
                f"Pitch Lock: {self.pitch_lock_commanded}\n"
                f"Yaw Lock: {self.yaw_lock_commanded}\n"
                f"Yaw Frame: {yaw_frame}")

    def _root_tfs(self, stamp) -> list[TransformStamped]:
        """Return the mutable raw-home and correction edges at ``stamp``."""
        if (not self.publish_fiducial_edge
                or not hasattr(self, '_home_lla') or not self._fiducial_lla):
            return []
        from copy import deepcopy
        raw, correction = deepcopy(self.raw_home_t), deepcopy(self.correction_t)
        raw.header.stamp = correction.header.stamp = stamp
        if hasattr(self, 'localization_reference'):
            state = self._reference_state(stamp)
            if state is None:
                return []
            correction.transform.translation = Vector3(
                x=state.correction[0], y=state.correction[1], z=state.correction[2])
        return [raw, correction]

    def update_fiducial(self, msg: TransformStamped):
        """Apply a correction-only raw-home -> corrected-home update.

        The translation is the correction E = surveyed - measured: a point sitting at `p` in
        the raw home frame belongs at `p + E` in the corrected home frame. New publishers name
        that edge directly. The old fiducial -> home parent is accepted for bag compatibility;
        its payload also contained only E despite the legacy frame name.

        Rotation is ignored; corrections are translation-only.
        """
        if not self.publish_fiducial_edge or self.external_reference_topic:
            return
        if (msg.header.frame_id not in (self.UNCORRECTED_HOME_FRAME,
                                        self.FIDUCIAL_FRAME)
                or msg.child_frame_id != self.HOME_FRAME):
            self.get_logger().warn(
                f"ignoring fiducial_update for {msg.header.frame_id} -> {msg.child_frame_id}; "
                f"expected {self.UNCORRECTED_HOME_FRAME} -> {self.HOME_FRAME}"
            )
            return

        old, new = self._fiducial_correction, msg.transform.translation
        if not np.all(np.isfinite([new.x, new.y, new.z])):
            return
        activation = stamp_ns(msg.header.stamp)
        if activation < getattr(self, '_correction_activation', -1):
            return
        self._correction_activation = activation
        self.localization_reference.update_correction(activation, (new.x, new.y, new.z))
        # tf_loc publishes a survey once and latches it, so this node is re-told the standing
        # correction whenever it restarts, and the update usually carries a value we already
        # hold. Re-broadcast regardless -- a restart comes up with an identity edge and this is
        # what puts the survey back -- but only log when the correction actually moved.
        changed = max(abs(new.x - old.x), abs(new.y - old.y), abs(new.z - old.z)) > 1e-6

        self._fiducial_correction = Vector3(x=new.x, y=new.y, z=new.z)
        self._compose_fiducial_edges()
        root_tfs = self._root_tfs(self.get_clock().now().to_msg())
        if root_tfs:
            self.tf_broadcaster.sendTransform(root_tfs)

        self._publish_reference(msg.header.stamp)
        if changed:
            self.get_logger().info(
                f"updated fiducial correction: ({new.x:+.2f}, {new.y:+.2f}, {new.z:+.2f})m"
            )
        else:
            self.get_logger().debug("fiducial transform re-asserted (unchanged)")

    def update_calibration(self, msg: FiducialCalibration):
        if msg.header.frame_id != self.UNCORRECTED_HOME_FRAME:
            return
        values = [msg.translation.x, msg.translation.y, msg.translation.z]
        q = msg.sensor_rotation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if (not np.all(np.isfinite(values)) or not np.all(np.isfinite(quaternion))
                or abs(np.linalg.norm(quaternion)-1) > 1e-3):
            self.get_logger().error('nonfinite fiducial calibration ignored')
            return
        update = TransformStamped()
        update.header = msg.header
        update.child_frame_id = self.HOME_FRAME
        update.transform.translation = msg.translation
        update.transform.rotation.w = 1.0
        self.update_fiducial(update)

    def update_alt(self, msg: Altitude):
        self.ALTITUDE=msg

    def update_velocity(self, msg: TwistStamped):
        self.VELOCITY=msg

    def publish_position(self, msg: PoseStamped):
        # header
        # TODO: double check time sync between message schemas
        head_out = Header(frame_id=self.PARENT_FRAME)

        path_update = PoseStamped()

        head_out.stamp = msg.header.stamp
        pos_in = msg.pose.position
        pos_out = Vector3(x=pos_in.x, y=pos_in.y, z=pos_in.z)
        tf_out = Transform(translation=pos_out, rotation=msg.pose.orientation)

        path_update.pose.position.x = float(pos_in.x)
        path_update.pose.position.y = float(pos_in.y)
        path_update.pose.position.z = float(pos_in.z)
        path_update.pose.orientation = msg.pose.orientation

        if self.VELOCITY:
            vel_in = self.VELOCITY.twist.linear
            self.drone_velocity = [float(vel_in.x), float(vel_in.y), float(vel_in.z)]

        new_pos = (float(pos_in.x), float(pos_in.y), float(pos_in.z))
        self.drone_pos = list(new_pos)

        if np.isfinite(self.BENCH_BASE_ALTITUDE):
            tf_out.translation.z = self.BENCH_BASE_ALTITUDE
            path_update.pose.position.z = self.BENCH_BASE_ALTITUDE
            new_pos = (new_pos[0], new_pos[1], self.BENCH_BASE_ALTITUDE)
            self.drone_pos = list(new_pos)

        path_update.header = head_out
        # keep the most recent header for downstream publishers
        self.latest_header = head_out

        # Every frame this callback builds shares one stamp, so they go out in
        # one message. Separate sends make one /tf message each and a listener
        # pays per message.
        tfs = self._root_tfs(head_out.stamp) + [TransformStamped(
            header=head_out, child_frame_id=self.FRAME_NAME, transform=tf_out
        )]

        # The home offset rides the pose rate so that it has a real time extent.
        # home_cb also sends once immediately to connect the tree at startup.
        if self.home_t is not None:
            state = self._reference_state(head_out.stamp)
            if state is not None:
                tfs.append(TransformStamped(
                    header=Header(stamp=head_out.stamp, frame_id=self.HOME_FRAME),
                    child_frame_id=self.EKF_FRAME,
                    transform=Transform(translation=Vector3(
                        x=state.ekf_offset[0], y=state.ekf_offset[1],
                        z=state.ekf_offset[2]))))

        # build PoseStamped for path
        # Path update
        if self.last_drone_pos is None or not self._positions_equal(
            self.last_drone_pos, new_pos, self.POSITION_TOLERANCE
        ):
            self.path.add(path_update)
            self.last_drone_pos = new_pos

        # publish the reference frame for a gimbal
        # construct the gimbal reference frame based on the active flags
        q = tf_out.rotation
        R_world_body = R.from_quat([q.x, q.y, q.z, q.w])
        gimbal_flags = self.gimbal_flags
        if not self.GIMBAL_HAS_YAW_AXIS:
            gimbal_flags = (gimbal_flags & ~(
                FLAGS_YAW_LOCK | FLAGS_YAW_IN_EARTH_FRAME))
            gimbal_flags |= FLAGS_YAW_IN_VEHICLE_FRAME
        R_body_ref = gimbal_reference_from_body(
            R_world_body,
            gimbal_flags,
            self.gimbal_reference_apply_stabilization_correction,
            self.gimbal_reference_yaw_is_earth,
            self.gimbal_reference_rotation)
        (q_x_ref, q_y_ref, q_z_ref, q_w_ref) = R_body_ref.as_quat()
        q_body_ref = Quaternion(x=q_x_ref, y=q_y_ref, z=q_z_ref, w=q_w_ref)
        # make and publish the transform
        g_ref = TransformStamped()
        g_ref.header = Header()
        g_ref.header.frame_id = self.gimbal_offset_frame
        g_ref.header.stamp = head_out.stamp
        g_ref.child_frame_id = self.gimbal_ref_frame
        g_ref.transform = Transform()
        g_ref.transform.rotation = q_body_ref
        tfs.append(g_ref)

        # Publish altimeter plane viz. investigation only, not required.
        # bottom_clearance is the height over whatever the rangefinder sees, and
        # PX4 reports it as NaN whenever nothing is in range, which is most of a
        # flight. Publishing that gives tf2 a NaN translation to reject at the
        # transform rate, so the frame waits for a reading instead.
        if self.ALTITUDE and np.isfinite(self.ALTITUDE.bottom_clearance):
            # a fresh Vector3, because a message field assigned from another
            # message holds the same object and lowering z here would lower the
            # body frame with it
            tfs.append(TransformStamped(
                header=head_out,
                child_frame_id=f"{self.FRAME_NAME}_alt_plane",
                transform=Transform(translation=Vector3(
                    x=tf_out.translation.x,
                    y=tf_out.translation.y,
                    z=tf_out.translation.z - self.ALTITUDE.bottom_clearance,
                ))
            ))

        self.tf_broadcaster.sendTransform(tfs)

    def _reference_state(self, stamp):
        return self.localization_reference.state(
            stamp_ns(stamp), self.UNCORRECTED_HOME_FRAME,
            self.HOME_FRAME, self.EKF_FRAME)

    def _publish_reference(self, stamp):
        state = self._reference_state(stamp)
        if state is None:
            return
        message = String(data=state.encode())
        self.reference_events_pub.publish(message)
        # A delayed event belongs in history, not in the latched latest state.
        latest_stamp = max(self.localization_reference.latest_home_stamp,
                           getattr(self, '_correction_activation', 0))
        latest = self.localization_reference.state(
            latest_stamp, self.UNCORRECTED_HOME_FRAME, self.HOME_FRAME, self.EKF_FRAME)
        self.reference_pub.publish(String(data=latest.encode()))

    def home_cb(self, msg: HomePosition):
        """Consume an atomic geographic/local pair, never separate home edges.

        Original PX4 navigation home remains on MAVROS home_position/home.
        home_position/fix describes the stable application HOME frame.
        """
        if self.external_reference_topic:
            return
        try:
            changed = self.localization_reference.update_home(
                stamp_ns(msg.header.stamp),
                (msg.geo.latitude, msg.geo.longitude, msg.geo.altitude),
                (msg.position.x, msg.position.y, msg.position.z))
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if not changed:
            return
        self._apply_reference(msg.header.stamp)

    def reference_cb(self, msg: String):
        try:
            state = ReferenceState.decode(msg.data)
            if (state.raw_frame, state.home_frame, state.ekf_frame) != (
                    self.UNCORRECTED_HOME_FRAME, self.HOME_FRAME, self.EKF_FRAME):
                return
            if not self.localization_reference.ingest(state):
                return
            from builtin_interfaces.msg import Time
            stamp = Time(sec=state.stamp // 1_000_000_000,
                         nanosec=state.stamp % 1_000_000_000)
            self._apply_reference(stamp)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().error(f'invalid onboard localization reference: {error}')

    def _apply_reference(self, stamp):
        anchor = self.localization_reference.anchor
        self._home_lla = NavSatFix(
            header=Header(frame_id=self.HOME_FRAME, stamp=stamp),
            latitude=anchor[0], longitude=anchor[1], altitude=anchor[2])
        self.home_fix_pub.publish(self._home_lla)
        if self._fiducial_lla and len(self._fiducial_lla) == 3:
            self._fiducial_fix_pub.publish(NavSatFix(
                header=Header(frame_id=self.FIDUCIAL_FRAME, stamp=stamp),
                latitude=float(self._fiducial_lla[0]),
                longitude=float(self._fiducial_lla[1]),
                altitude=float(self._fiducial_lla[2])))
        self._compose_fiducial_edges()
        state = self._reference_state(stamp)
        self.home_t = TransformStamped(
            header=Header(stamp=stamp, frame_id=self.HOME_FRAME),
            child_frame_id=self.EKF_FRAME,
            transform=Transform(translation=Vector3(
                x=state.ekf_offset[0], y=state.ekf_offset[1], z=state.ekf_offset[2])))
        self.tf_broadcaster.sendTransform(
            self._root_tfs(stamp) + [self.home_t])
        self._publish_reference(stamp)
        lat, lon, alt = state.frame_anchor(self.EKF_FRAME, corrected=False)
        self.ekf_fix_pub.publish(NavSatFix(
            header=Header(frame_id=self.EKF_FRAME, stamp=stamp),
            latitude=lat, longitude=lon, altitude=alt))

    def _compose_fiducial_edges(self):
        """Place raw HOME from GPS and publish the survey as its own edge."""
        if not hasattr(self, '_home_lla'):
            return
        if not self._fiducial_lla or len(self._fiducial_lla) != 3:
            return
        fid = self._fiducial_lla
        base = pm.geodetic2enu(self._home_lla.latitude, self._home_lla.longitude,
                               self._home_lla.altitude, fid[0], fid[1], fid[2], deg=True)
        self.raw_home_t.transform.translation.x = float(base[0])
        self.raw_home_t.transform.translation.y = float(base[1])
        self.raw_home_t.transform.translation.z = float(base[2])
        self.raw_home_t.transform.rotation.w = 1.0
        self.correction_t.transform.translation = Vector3(
            x=self._fiducial_correction.x,
            y=self._fiducial_correction.y,
            z=self._fiducial_correction.z)
        self.correction_t.transform.rotation.w = 1.0
        stamp = self.get_clock().now().to_msg()
        self.raw_home_t.header.stamp = stamp
        self.correction_t.header.stamp = stamp

    def publish_path(self):
        message = self.path.message()
        if message.markers:
            self.path_pub.publish(message)

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

        self.velocity_vector_pub.publish(velocity_vector_marker)

    def position_conversion(self, x_in:float, y_in:float, z_in:float) -> Vector3:
        if 'ned' in self.POSE_FRAME:
            return Vector3(x=y_in, y=x_in, z=-z_in)
        elif 'enu' in self.POSE_FRAME:
            return Vector3(x=x_in, y=y_in, z=z_in)
        else:
            raise ValueError(f"Unable to determine the coordinate frame for message type: {self.POSE_FRAME}")

    @staticmethod
    def _positions_equal(a: Tuple[float, float, float], b: Tuple[float, float, float], tol: float = 1e-6) -> bool:
        return all(isclose(x, y, rel_tol=0.0, abs_tol=tol) for x, y in zip(a, b))

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
