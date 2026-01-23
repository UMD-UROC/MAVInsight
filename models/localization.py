# python imports
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from cdcl_umd_msgs.msg import TargetBoxArray, TargetBox
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

# MAVInsight imports
from models.frame_utils import frd_2_flu, lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import viz_qos, reliable_qos

class Localization(GraphMember):

    def __init__(self):
        super().__init__()

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Localization params....")

        if self.has_parameter('localization_topic'):
            loc_topic = self.get_parameter('localization_topic').get_parameter_value().string_value
        else:
            raise RuntimeError(f"Localization viz node: {self.DISPLAY_NAME} localization topic param not set. Unable to initialize localization vizualization.")

        if self.has_parameter('loc_viz_topic'):
            loc_viz_topic = self.get_parameter('loc_viz_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning('loc_viz_topic')
            loc_viz_topic = 'localization_viz'

        if self.has_parameter('map_ref'): # TODO: Make this dynamic/smarter (i hate this)
            map_ref_param = self.get_parameter('map_ref').get_parameter_value().double_array_value
            self.REF_FIX = NavSatFix(latitude=map_ref_param[0], longitude=map_ref_param[1], altitude=map_ref_param[2])
        else:
            raise RuntimeError(f"Localization viz node: {self.DISPLAY_NAME} map reference param not set. Unable to initialize node.")
        self.create_subscription(TargetBoxArray, loc_topic, self.loc_cb, reliable_qos)
        self.pub = self.create_publisher(MarkerArray, loc_viz_topic, reliable_qos)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Localization Visualization initialized!")

    def loc_cb(self, msg: TargetBoxArray):
        self.get_logger().debug("tba received")
        drone_pose = msg.uav_local_pose.pose.pose
        drone_marker = Marker(
            header=Header(frame_id="map"), #TODO: Make the origin frame accessible to all Graph Members
            ns="drone",
            id=0,
            pose=drone_pose,
            type=Marker.CUBE,
            action=Marker.ADD,
            scale=Vector3(x=0.5, y=0.56, z=0.24),
            color=ColorRGBA(r=153.0/255.0, g=153.0/255.0, b=153.0/255.0, a=0.9)
        )

        drone_q = drone_pose.orientation
        drone_r = R.from_quat([drone_q.x, drone_q.y, drone_q.z, drone_q.w])
        (dr_x, dr_y, dr_z) = drone_r.as_euler("xyz", degrees=True)

        gimbal_q_frd = msg.gimbal_attitude_quaternion
        gimbal_r_frd = R.from_quat([gimbal_q_frd.x, gimbal_q_frd.y, gimbal_q_frd.z, gimbal_q_frd.w])
        gimbal_r_enu = frd_2_flu(gimbal_r_frd)
        assert(isinstance(gimbal_r_enu, R))
        (gr_x, gr_y, gr_z) = gimbal_r_enu.as_euler('xyz', degrees=True)

        R_world_gimbal = R.from_euler('yz', [gr_y, dr_z], degrees=True)
        (x, y, z, w) = R_world_gimbal.as_quat()

        rangefinder_marker = Marker(
            header=Header(frame_id="map"),
            ns="rangefinder",
            id=0,
            type=Marker.ARROW,
            action=Marker.ADD,
            pose = Pose(
                position=drone_pose.position,
                orientation=Quaternion(x=x, y=y, z=z, w=w)
            ),
            scale=Vector3(x=msg.rangefinder_dist.range, y=0.1, z=0.1),
            color=ColorRGBA(r=86.0/255.0, g=209.0/255.0, b=86.0/255.0, a=0.75)
        )

        markers = [drone_marker, rangefinder_marker]

        rangefinder_fixes = []
        gimbal_plane_fixes = []
        altimeter_plane_fixes = []

        for box in msg.uav_target_boxes:
            i = 0
            self.get_logger().debug("found boxes")
            assert(isinstance(box, TargetBox))
            self.get_logger().debug(f"processing box: {i}")
            if box.target_location_altimeter_plane:
                loc: NavSatFix = box.target_location_altimeter_plane
                altimeter_plane_fixes.append(lla_2_enu(self.REF_FIX, loc))

            if box.target_location_gimbal_plane:
                loc: NavSatFix = box.target_location_gimbal_plane
                gimbal_plane_fixes.append(lla_2_enu(self.REF_FIX, loc))

            if box.target_location_rangefinder:
                loc: NavSatFix = box.target_location_rangefinder
                rangefinder_fixes.append(lla_2_enu(self.REF_FIX, loc))

        rangefinder_points = [Point(x=e, y=n, z=u) for e, n, u in rangefinder_fixes]
        gimbal_plane_points = [Point(x=e, y=n, z=u) for e, n, u in gimbal_plane_fixes]
        altimeter_plane_points = [Point(x=e, y=n, z=u) for e, n, u in altimeter_plane_fixes]

        markers.append(Marker(
            header=Header(frame_id="map"),
            ns="rangefinder_loczn",
            id=0,
            type=Marker.CUBE_LIST,
            action=Marker.ADD,
            points=rangefinder_points,
            scale=Vector3(x=0.25, y=0.25, z=0.25),
            color=ColorRGBA(r=255.0/255.0, g=0.0/255.0, b=0.0/255.0, a=0.75)
        ))

        markers.append(Marker(
            header=Header(frame_id="map"),
            ns="gimbal_plane_loczn",
            id=0,
            type=Marker.SPHERE_LIST,
            action=Marker.ADD,
            points=gimbal_plane_points,
            scale=Vector3(x=0.25, y=0.25, z=0.25),
            color=ColorRGBA(r=0.0/255.0, g=255.0/255.0, b=0.0/255.0, a=0.75)
        ))

        markers.append(Marker(
            header=Header(frame_id="map"),
            ns="alt_plane_loczn",
            id=0,
            type=Marker.POINTS,
            action=Marker.ADD,
            points=altimeter_plane_points,
            scale=Vector3(x=0.25, y=0.25, z=0.25),
            color=ColorRGBA(r=255.0/255.0, g=248.0/255.0, b=51.0/255.0, a=0.75)
        ))

        self.pub.publish(MarkerArray(markers=markers))
