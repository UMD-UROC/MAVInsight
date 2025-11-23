# python imports
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from geometry_msgs.msg import Quaternion,Transform, TransformStamped
from std_msgs.msg import Header

def build_ned_2_enu_frame(parent_frame : str, child_frame : str):
    header = Header(frame_id=parent_frame)
    (x, y, z, w) = R.from_euler('xyz', [180.0, 0.0, 0.0], degrees=True).as_quat()
    ros_quat = Quaternion(x=x, y=y, z=z, w=w)

    transform = Transform(rotation=ros_quat)

    return TransformStamped(
        header=header,
        child_frame_id=child_frame,
        transform=transform
    )
