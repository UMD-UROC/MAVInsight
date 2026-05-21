# python imports
import math
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from cdcl_umd_msgs.msg import TargetBoxArray, TargetBox
from foxglove_msgs.msg import ImageAnnotations, Point2, PointsAnnotation, TextAnnotation
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import BoundingBox2D
from visualization_msgs.msg import Marker, MarkerArray

# MAVInsight imports
from models.frame_utils import frd_2_flu, lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import viz_qos, reliable_qos

class TBA_Viz(GraphMember):

    def __init__(self):
        super().__init__()

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Localization params....")

        if self.has_parameter('localization_topic'):
            loc_topic = self.get_parameter('localization_topic').get_parameter_value().string_value
        else:
            raise RuntimeError(f"Localization viz node: {self.DISPLAY_NAME} localization topic param not set. Unable to initialize localization vizualization.")

        if self.has_parameter('loc_viz_topic_latest'):
            loc_viz_topic_latest = self.get_parameter('loc_viz_topic_latest').get_parameter_value().string_value
        else:
            self.default_parameter_warning('loc_viz_topic_latest')
            loc_viz_topic_latest = '/viz/tbas/latest'

        if self.has_parameter('loc_viz_topic_previous'):
            loc_viz_topic_previous = self.get_parameter('loc_viz_topic_previous').get_parameter_value().string_value
        else:
            self.default_parameter_warning('loc_viz_topic_previous')
            loc_viz_topic_previous = '/viz/tbas/previous'

        if self.has_parameter('localization_frame'):
            self.LOC_FRAME = self.get_parameter('localization_frame').get_parameter_value().string_value
        else:
            raise RuntimeError(f"Localization viz node: {self.DISPLAY_NAME} localization frame param not set. Unable to initialize localization vizualization.")

        if self.has_parameter("mesh_resource_path"):
            self.mesh_resource_path = self.get_parameter("mesh_resource_path").get_parameter_value().string_value
            self.get_logger().info(f"Mesh Resource Path detected: {self.mesh_resource_path}")
        else:
            self.mesh_resource_path = None

        # marker color
        r, g, b, a = self.get_parameter('marker_color_rgba').get_parameter_value().double_array_value
        self.color_profile = ColorRGBA(r=(r/255.0), g=(g/255.0), b=(b/255.0), a=a)
        self.create_subscription(TargetBoxArray, loc_topic, self.loc_cb, reliable_qos)
        self.latest_pub = self.create_publisher(MarkerArray, loc_viz_topic_latest, reliable_qos)
        self.previous_pub = self.create_publisher(MarkerArray, loc_viz_topic_previous, reliable_qos)

        self.i = 0
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Localization Visualization initialized!")

    def loc_cb(self, msg: TargetBoxArray):
        self.get_logger().info("tba received")

        if msg.source_img.header.stamp.sec == 0:
            self.get_logger().error("AAAAA")
            return

        if msg.uav_target_boxes == None or len(msg.uav_target_boxes) == 0:
            self.get_logger().error("AAAAA2")
            return

        drone_pose = msg.uav_local_pose.pose.pose
        drone_marker = Marker()
        drone_marker.header=Header(frame_id=self.LOC_FRAME)
        drone_marker.ns="drone"
        drone_marker.id=0
        drone_marker.pose.position.x=drone_pose.position.x
        drone_marker.pose.position.y=drone_pose.position.y
        drone_marker.pose.position.z=drone_pose.position.z
        drone_marker.pose.orientation.x=drone_pose.orientation.x
        drone_marker.pose.orientation.y=drone_pose.orientation.y
        drone_marker.pose.orientation.z=drone_pose.orientation.z
        drone_marker.pose.orientation.w=drone_pose.orientation.w
        drone_marker.type=Marker.MESH_RESOURCE
        drone_marker.mesh_resource=self.mesh_resource_path
        drone_marker.mesh_use_embedded_materials=True
        drone_marker.action=Marker.ADD
        drone_marker.scale=Vector3(x=0.001, y=0.001, z=0.001)
        drone_marker.color=self.color_profile

        R_fix = R.from_euler('xyz', [0.0, 0.0, -math.pi/2], degrees=False)
        q = drone_marker.pose.orientation
        R_drone = R.from_quat([q.x, q.y, q.z, q.w])

        (x_fix, y_fix, z_fix, w_fix) = (R_fix * R_drone).as_quat()
        drone_marker.pose.orientation.x = x_fix
        drone_marker.pose.orientation.y = y_fix
        drone_marker.pose.orientation.z = z_fix
        drone_marker.pose.orientation.w = w_fix

        drone_q = drone_pose.orientation
        drone_r = R.from_quat([drone_q.x, drone_q.y, drone_q.z, drone_q.w])
        (dr_x, dr_y, dr_z) = drone_r.as_euler("xyz", degrees=True)

        gimbal_q_frd = msg.gimbal_attitude_quaternion
        gimbal_r_frd = R.from_quat([gimbal_q_frd.x, gimbal_q_frd.y, gimbal_q_frd.z, gimbal_q_frd.w])
        gimbal_r_enu = frd_2_flu(gimbal_r_frd)
        assert(isinstance(gimbal_r_enu, R))
        (gr_x, gr_y, gr_z) = gimbal_r_enu.as_euler('xyz', degrees=True)

        R_world_gimbal = R.from_euler('yz', [float(gr_y), float(dr_z)], degrees=True)
        (x, y, z, w) = R_world_gimbal.as_quat()

        rangefinder_marker = Marker(
            header=Header(frame_id=self.LOC_FRAME),
            ns="rangefinder",
            id=0,
            type=Marker.ARROW,
            action=Marker.ADD,
            pose = Pose(
                position=drone_pose.position,
                orientation=Quaternion(x=x, y=y, z=z, w=w)
            ),
            scale=Vector3(x=msg.rangefinder_dist.range, y=0.1, z=0.1),
            color=self.color_profile
        )

        markers = [drone_marker, rangefinder_marker]

        rangefinder_fixes = []
        gimbal_plane_fixes = []
        altimeter_plane_fixes = []

        for box in msg.uav_target_boxes:
            assert(isinstance(box, TargetBox))
            if box.target_location_altimeter_plane:
                loc: NavSatFix = box.target_location_altimeter_plane
                e, n, u = lla_2_enu(msg.uav_gps_location, loc, ignore_alt=False)
                e += msg.uav_local_pose.pose.pose.position.x
                n += msg.uav_local_pose.pose.pose.position.y
                u += msg.uav_local_pose.pose.pose.position.z
                altimeter_plane_fixes.append((e, n, u))

            if box.target_location_gimbal_plane:
                loc: NavSatFix = box.target_location_gimbal_plane
                e, n, u = lla_2_enu(msg.uav_gps_location, loc)
                e += msg.uav_local_pose.pose.pose.position.x
                n += msg.uav_local_pose.pose.pose.position.y
                u += msg.uav_local_pose.pose.pose.position.z
                gimbal_plane_fixes.append((e, n, u))

            if box.target_location_rangefinder:
                loc: NavSatFix = box.target_location_rangefinder
                e, n, u = lla_2_enu(msg.uav_gps_location, loc)
                e += msg.uav_local_pose.pose.pose.position.x
                n += msg.uav_local_pose.pose.pose.position.y
                u += msg.uav_local_pose.pose.pose.position.z
                rangefinder_fixes.append((e, n, u))

        rangefinder_points = [Point(x=e, y=n, z=u) for e, n, u in rangefinder_fixes]
        gimbal_plane_points = [Point(x=e, y=n, z=u) for e, n, u in gimbal_plane_fixes]
        altimeter_plane_points = [Point(x=e, y=n, z=u) for e, n, u in altimeter_plane_fixes]

        markers.append(Marker(
            header=Header(frame_id=self.LOC_FRAME),
            ns="alt_plane_last",
            id=0,
            type=Marker.SPHERE_LIST,
            action=Marker.ADD,
            points=altimeter_plane_points,
            scale=Vector3(x=0.25, y=0.25, z=0.25),
            color=self.color_profile
        ))

        self.latest_pub.publish(MarkerArray(markers=markers))
        self.get_logger().debug("Published latest")

        [latest_to_previous(m, self.i) for m in markers]
        self.previous_pub.publish(MarkerArray(markers=markers))

        self.i+=1

def latest_to_previous(input:Marker, i: int):
    input.id = i
    input.ns = input.ns.replace("last", "all")
    return input
