# python imports
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from geometry_msgs.msg import Quaternion

# define the NED -> ENU world frame conversion as a remapping of axes
R_ned_enu = R.from_matrix(np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0,-1],
]))

R_enu_ned = R_ned_enu.inv()

# define the FRD -> FLU body frame conversions as a remapping of axes (symmetric in this case)
R_frd_flu = R.from_matrix(np.array([
    [1, 0, 0],
    [0,-1, 0],
    [0, 0,-1]
]))

def frd_2_flu(input):
    if isinstance(input, Quaternion):
        r = R.from_quat([input.x, input.y, input.z, input.w])
        (x_out, y_out, z_out, w_out) = (R_frd_flu * r * R_frd_flu).as_quat()
        return Quaternion(x=x_out, y=y_out, z=z_out, w=w_out)
    elif isinstance(input, R):
        return R_frd_flu * input * R_frd_flu
    else:
        raise ValueError(f"Unrecognized input type in frd_2_flu conversion: {input}")

def frd_ned_2_flu_enu(input):
    if isinstance(input, Quaternion):
        r = R.from_quat([input.x, input.y, input.z, input.w])
        (x_out, y_out, z_out, w_out) = ((R_ned_enu * r * R_frd_flu)).as_quat() #NOTE: TF2 quaternions are child->parent
        return Quaternion(x=x_out, y=y_out, z=z_out, w=w_out)
    elif isinstance(input, R):
        return R_frd_flu * input * R_enu_ned
    else:
        raise ValueError(f"Unrecognized input type in frd_ned_2_enu_flu conversion: {input}")
