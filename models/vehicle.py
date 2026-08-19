# python imports
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Transform, TransformStamped, TwistStamped, Vector3
from mavros_msgs.msg import Altitude, HomePosition, GimbalDeviceAttitudeStatus
# from nav_msgs.msg import Path
from px4_msgs.msg import VehicleOdometry  # type:ignore
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

# MAVInsight imports
from models.frame_utils import R_enu_nwu, enu_2_lla, frd_ned_2_flu_enu
from models.frame_member import FrameMember
from models.platforms import Platforms
from models.qos_profiles import reliable_qos, viz_qos

FLAGS_RETRACT = 1
FLAGS_NEUTRAL = 2
FLAGS_ROLL_LOCK = 4
FLAGS_PITCH_LOCK = 8
FLAGS_YAW_LOCK = 16

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
            fiducial_frame = self.get_parameter("fiducial_frame").get_parameter_value().string_value
        else:
            fiducial_frame = "fiducial"

        if self.has_parameter("fiducial_update_topic"):
            fiducial_update_topic = self.get_parameter("fiducial_update_topic").get_parameter_value().string_value
        else:
            fiducial_update_topic = "fiducial_update"

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
        self.create_subscription(Altitude, alt_topic, self.update_alt, viz_qos)
        self.create_subscription(self.LOCATION_MSG_TYPE, self.LOCATION_TOPIC, self.publish_position, viz_qos)
        self.create_subscription(HomePosition, home_pos_topic, self.home_cb, viz_qos)
        self.create_subscription(TwistStamped, velocity_topic, self.update_velocity, viz_qos)
        self.create_subscription(TransformStamped, fiducial_update_topic, self.update_fiducial, reliable_qos)
        self.ALTITUDE = None
        self.VELOCITY = None

        # Initialize publishers
        #self.path_pub = self.create_publisher(Path, f"{namespace}flightPath", reliable_qos)
        self.home_fix_pub = self.create_publisher(NavSatFix, home_fix_topic, reliable_qos)
        self.ekf_fix_pub = self.create_publisher(NavSatFix, ekf_topic, reliable_qos)
        self.velocity_vector_pub = self.create_publisher(Marker, f"{namespace}velocityVector", reliable_qos)

        # Internal storage for path visualizer
        #self.path = Path()
        #self.path.header.frame_id = self.PARENT_FRAME
        self.latest_pose = None

        # Initialize state variables for velocity and position tracking
        self.drone_velocity = [0.0, 0.0, 0.0]  # Current velocity (m/s)
        self.drone_pos = [0.0, 0.0, 0.0]  # Current position (m)
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

        self.create_subscription(GimbalDeviceAttitudeStatus, gimbal_flags_topic, self.update_gimbal_flags, viz_qos)
        # initialize gimbal state variables
        self.retract_commanded = False
        self.neutral_position_commanded = False
        self.roll_lock_commanded = False
        self.pitch_lock_commanded = False
        self.yaw_lock_commanded = False

        # Publisher timers
        #self.create_timer(1.0 / self.REFRESH_RATE, self.publish_path)
        self.create_timer(1.0 / self.REFRESH_RATE, self.publish_velocity_vector)
        self.create_timer(10.0, self.publish_static_tfs)

        # fiducial -> home: the shared fiducial frame parents every vehicle's home position
        self.fid_t = TransformStamped()
        self.fid_t.header = Header(frame_id=fiducial_frame, stamp=self.get_clock().now().to_msg())
        self.fid_t.child_frame_id = self.HOME_FRAME

        # home -> ekf origin offset, kept static between the sparse home position
        # updates (mavros can go minutes without republishing home_position/home,
        # and a dynamic transform stamped only at those updates leaves the whole
        # chain un-lookupable in between)
        self.home_t = None

        # seed the tree with the fiducial frame now rather than at the first 10s
        # timer tick -- consumers expect it to be present from startup
        self.tf_static_broadcaster.sendTransform(self._static_tfs())

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Vehicle initialized!")

    def update_gimbal_flags(self, msg: GimbalDeviceAttitudeStatus):
        flags = int(msg.flags)
        new_flag = False
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
        if new_flag:
            self.get_logger().info(f"~~~~~~~NEW Gimbal Flags~~~~~~~~\nRetract: {self.retract_commanded}\nNeutral: {self.neutral_position_commanded}\nRoll Lock: {self.roll_lock_commanded}\nPitch Lock: {self.pitch_lock_commanded}\nYaw Lock: {self.yaw_lock_commanded}")

    def publish_static_tfs(self):
        # timer cb to occasionaly publish static tfs for late joiners
        # self.fid_t.header.stamp = self.get_clock().now().to_msg() # commenting out for data playback, maybe not needed
        self.tf_static_broadcaster.sendTransform(self._static_tfs())

    def _static_tfs(self) -> list[TransformStamped]:
        # sent together in one message: the static broadcaster latches with depth 1,
        # so a late joiner only ever sees the last message sent
        return [self.fid_t] + ([self.home_t] if self.home_t is not None else [])

    def update_fiducial(self, msg: TransformStamped):
        """`fiducial_update` IS this vehicle's fiducial -> home edge, applied verbatim.

        The translation is the correction E = surveyed - measured: a point sitting at `p` in
        the raw home frame belongs at `p + E` in the fiducial frame. tf_loc publishes exactly
        that, so there is no sign flip here -- the header and the payload agree, and the edge
        can be read straight off the wire.

        Rotation is ignored; corrections are translation-only.
        """
        if msg.header.frame_id != self.fid_t.header.frame_id or msg.child_frame_id != self.HOME_FRAME:
            self.get_logger().warn(
                f"ignoring fiducial_update for {msg.header.frame_id} -> {msg.child_frame_id}; "
                f"this vehicle publishes {self.fid_t.header.frame_id} -> {self.HOME_FRAME}"
            )
            return

        old, new = self.fid_t.transform.translation, msg.transform.translation
        # tf_loc re-asserts the standing correction on a timer so the edge survives a restart
        # here, so most updates carry a value we already hold. Re-broadcast regardless (that is
        # the point of the re-assert), but only log when the correction actually moved --
        # otherwise this is a log line every republish period, forever.
        changed = max(abs(new.x - old.x), abs(new.y - old.y), abs(new.z - old.z)) > 1e-6

        self.fid_t.transform.translation.x = new.x
        self.fid_t.transform.translation.y = new.y
        self.fid_t.transform.translation.z = new.z
        self.fid_t.header.stamp = self.get_clock().now().to_msg()
        self.tf_static_broadcaster.sendTransform(self._static_tfs())

        if changed:
            self.get_logger().info(
                f"updated fiducial transform: ({new.x:+.2f}, {new.y:+.2f}, {new.z:+.2f})m"
            )
        else:
            self.get_logger().debug("fiducial transform re-asserted (unchanged)")

    def update_alt(self, msg: Altitude):
        self.ALTITUDE=msg

    def update_velocity(self, msg: TwistStamped):
        self.VELOCITY=msg

    def publish_position(self, msg: PoseStamped | VehicleOdometry):
        # header
        # TODO: double check time sync between message schemas
        head_out = Header(frame_id=self.PARENT_FRAME)

        #path_update = PoseStamped()

        # transform
        if self.LOCATION_MSG_TYPE == VehicleOdometry:
            assert isinstance(msg, VehicleOdometry)
            # TODO: Header needs the timestamp from the PX4 Message. Include when uXRCE is introduced.
            pos_in = msg.position

            pos_out = self.position_conversion(x_in=float(pos_in[0]), y_in=float(pos_in[1]), z_in=float(pos_in[2]))
            q_out_frd = Quaternion(x=float(msg.q[1]), y=float(msg.q[2]), z=float(msg.q[3]), w=float(msg.q[0]))

            q_out_flu = frd_ned_2_flu_enu(q_out_frd)
            tf_out = Transform(translation=pos_out, rotation=q_out_flu)

            # path_update.pose.position.x = pos_out.x
            # path_update.pose.position.y = pos_out.y
            # path_update.pose.position.z = pos_out.z
            # path_update.pose.orientation = q_out_flu

            # TODO: Migrate to new function
            vel_in = msg.velocity
            vel_out = self.position_conversion(x_in=float(vel_in[0]), y_in=float(vel_in[1]), z_in=float(vel_in[2]))
            self.drone_velocity = [vel_out.x, vel_out.y, vel_out.z]

            self.drone_pos = [pos_out.x, pos_out.y, pos_out.z]

        else:
            assert isinstance(msg, PoseStamped)
            head_out.stamp = msg.header.stamp
            pos_in = msg.pose.position
            pos_out = Vector3(x=pos_in.x, y=pos_in.y, z=pos_in.z)
            tf_out = Transform(translation=pos_out, rotation=msg.pose.orientation)

            # path_update.pose.position.x = float(pos_in.x)
            # path_update.pose.position.y = float(pos_in.y)
            # path_update.pose.position.z = float(pos_in.z)
            # path_update.pose.orientation = msg.pose.orientation

            # Extract velocity from Odometry message
            if self.VELOCITY:
                vel_in = self.VELOCITY.twist.linear
                self.drone_velocity = [float(vel_in.x), float(vel_in.y), float(vel_in.z)]

            self.drone_pos = [float(pos_in.x), float(pos_in.y), float(pos_in.z)]

        # path_update.header = head_out

        # keep the most recent header for downstream publishers
        self.latest_header = head_out

        # build TF
        t = TransformStamped(
            header=head_out, child_frame_id=self.FRAME_NAME, transform=tf_out
        )

        self.tf_broadcaster.sendTransform(t)

        # build PoseStamped for path
        # Path update
        # self.path.poses.append(path_update) # type: ignore
        # self.path.header.stamp = path_update.header.stamp

        # publish the reference frame for a gimbal
        # construct the gimbal reference frame based on the active flags
        R_body_ref = R.identity()
        if self.roll_lock_commanded or self.pitch_lock_commanded or self.yaw_lock_commanded:
            q = tf_out.rotation
            R_world_body = R.from_quat([q.x, q.y, q.z, q.w])
            R_body_ref *= R_world_body.inv()
            # A locked axis is measured against a level frame. The yaw lock picks
            # which level frame: the earth frame, whose x axis points North, or the
            # vehicle frame, whose x axis follows the airframe heading. MAVLink
            # gives both in NED, so the x axis of the earth frame is North, which
            # is 90 degrees from the x axis of this ENU tree. Drop that turn and a
            # yaw-locked gimbal builds a camera frame 90 degrees off, on real
            # hardware as much as in simulation.
            if self.yaw_lock_commanded:
                R_body_ref *= R_enu_nwu
            else:
                R_body_ref *= self.heading_only_frame(tf_out.rotation)
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
        self.tf_broadcaster.sendTransform(g_ref)

        # Publish altimeter plane viz. investigation only, not required.
        # bottom_clearance is the height over whatever the rangefinder sees, and
        # PX4 reports it as NaN whenever nothing is in range, which is most of a
        # flight. Publishing that gives tf2 a NaN translation to reject at the
        # transform rate, so the frame waits for a reading instead.
        if self.ALTITUDE and np.isfinite(self.ALTITUDE.bottom_clearance):
            alt_t = Transform(translation=tf_out.translation)
            alt_t.translation.z -= self.ALTITUDE.bottom_clearance
            self.tf_broadcaster.sendTransform(TransformStamped(
                header=head_out,
                child_frame_id=f"{self.FRAME_NAME}_alt_plane",
                transform=alt_t
            ))

    def home_cb(self, msg: HomePosition):
        """
        TODO: If the home position is going to be in our frame tree, then we need to
        better understand the implications, and make sure this is being TIME SYNCED
        properly.
        Current:
            Using the home position's local coordinates to back-out the local EKF
            Origin so that we can use the EKF-centered local position estimate in
            our tree. In theory, this allows us capture ekf corrections, which only
            present obviously in changes to the home position. Other benefits
            include better viz for the TRUE local position origin and home position
            viz.
        Alternative:
            Just ground the Local position estimate as the authoritative loczn position
            estimate.
            OR
            Use the Home-position grounded local position estimate and chain
            off of that. (no-op, really)
        TIME SYNC CONSIDERATION:
            DTC rosbags show home position is only published @ .5Hz,but home position
            correction seems infrequent. SEE: Foxglove PLOT viz of local coords of home.
        """
        home_fix = NavSatFix(
            header=Header(frame_id=self.HOME_FRAME, stamp=msg.header.stamp),
            latitude=msg.geo.latitude,
            longitude=msg.geo.longitude,
            altitude=msg.geo.altitude
        )
        self.home_fix_pub.publish(home_fix)
        # broadcast as static: the offset is piecewise-constant between home
        # updates, and a static transform stays valid for lookups at any stamp
        # (same treatment as the fiducial offset above)
        self.home_t = TransformStamped(
            header=Header(stamp=msg.header.stamp, frame_id=self.HOME_FRAME),
            child_frame_id=self.EKF_FRAME,
            transform=Transform(translation=Vector3(x=-msg.position.x, y=-msg.position.y, z=-msg.position.z))
        )
        self.tf_static_broadcaster.sendTransform(self._static_tfs())

        (lat_e, lon_e, alt_e) = enu_2_lla(home_fix, -msg.position.x, -msg.position.y, -msg.position.z)
        self.ekf_fix_pub.publish(NavSatFix(
            header=Header(frame_id=self.EKF_FRAME, stamp=msg.header.stamp),
            latitude=lat_e,
            longitude=lon_e,
            altitude=alt_e
        ))

    # def publish_path(self):
    #     if self.path.poses:
    #         self.path_pub.publish(self.path)

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

    def heading_only_frame(self, o:Quaternion) -> R:
        R_world_body = R.from_quat([o.x, o.y, o.z, o.w])
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
