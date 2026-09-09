"""Resolve MAVLink gimbal reports into the ROS frame tree."""

import numpy as np
from scipy.spatial.transform import Rotation as R


FLAGS_RETRACT = 1
FLAGS_NEUTRAL = 2
FLAGS_ROLL_LOCK = 4
FLAGS_PITCH_LOCK = 8
FLAGS_YAW_LOCK = 16
FLAGS_YAW_IN_VEHICLE_FRAME = 32
FLAGS_YAW_IN_EARTH_FRAME = 64
LOCK_FLAGS = FLAGS_ROLL_LOCK | FLAGS_PITCH_LOCK | FLAGS_YAW_LOCK
EARTH_NORTH_IN_ENU = R.from_euler("Z", 90.0, degrees=True)


def yaw_is_earth_referenced(flags: int) -> bool:
    """Prefer explicit MAVLink yaw-frame flags over the legacy lock bit."""
    explicit_earth_frame = bool(flags & FLAGS_YAW_IN_EARTH_FRAME)
    explicit_vehicle_frame = bool(flags & FLAGS_YAW_IN_VEHICLE_FRAME)
    if explicit_earth_frame != explicit_vehicle_frame:
        return explicit_earth_frame
    return bool(flags & FLAGS_YAW_LOCK)


def leveled_vehicle_heading(body_orientation: R) -> R:
    """Return a level frame whose x axis follows the vehicle nose."""
    heading_vector = body_orientation.apply([0.0, 0.0, 0.0])
    heading_vector[2] = 0.0
    horizontal_norm = np.linalg.norm(heading_vector)
    if horizontal_norm < 1e-9:
        return R.identity()
    heading_vector /= horizontal_norm
    return R.from_euler("Z", np.arctan2(heading_vector[1], heading_vector[0]))


def gimbal_reference_from_body(body_orientation: R, flags: int) -> R:
    """Return the body-to-reference rotation declared by gimbal flags."""
    if not flags & LOCK_FLAGS:
        return R.identity()
    if yaw_is_earth_referenced(flags):
        leveled_reference = EARTH_NORTH_IN_ENU
    else:
        leveled_reference = leveled_vehicle_heading(body_orientation)
    return body_orientation.inv() * leveled_reference
