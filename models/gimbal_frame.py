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
    heading_vector = body_orientation.apply([1.0, 0.0, 0.0])
    heading_vector[2] = 0.0
    horizontal_norm = np.linalg.norm(heading_vector)
    if horizontal_norm < 1e-9:
        return R.from_euler("Z", 90, degrees=True)
    heading_vector /= horizontal_norm
    heading = np.arctan2(heading_vector[1], heading_vector[0])
    return R.from_euler("Z", heading)


def without_reported_yaw(attitude: R) -> R:
    """Return a v2 gimbal's pitch-only attitude in its body reference frame.

    Chimera v2 has a roll axis, but flight control deliberately commands that
    axis level (``ROLL_ANGLE == 0``). Its MAVLink attitude report nevertheless
    carries a yaw component even though the mount has no yaw actuator. The
    report cannot recover a separate roll angle at the pitch poles, so retain
    the commanded pitch and remove the non-physical yaw entirely.
    """
    q = attitude.as_quat()
    pitch = np.arctan2(
        2.0 * (q[3] * q[1] - q[2] * q[0]),
        1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]))
    return R.from_quat([0.0, np.sin(pitch / 2.0), 0.0, np.cos(pitch / 2.0)])


def gimbal_reference_from_body(
        body_orientation: R,
        flags: int,
        apply_stabilization_correction: bool = True,
        yaw_is_earth_referenced_override: bool | None = None,
        reference_rotation: R | None = None) -> R:
    """Return the body-to-reference rotation declared by gimbal flags.

    ``reference_rotation`` is an airframe-specific rotation from the gimbal
    offset frame to its attitude reference frame.  A yaw-frame override is for
    hardware whose attitude report uses an earth frame despite its MAVLink
    flags claiming a vehicle frame.
    """
    if reference_rotation is None:
        reference_rotation = R.identity()

    if not apply_stabilization_correction or not flags & LOCK_FLAGS:
        return reference_rotation
    earth_referenced = (yaw_is_earth_referenced(flags)
                        if yaw_is_earth_referenced_override is None
                        else yaw_is_earth_referenced_override)
    if earth_referenced:
        leveled_reference = EARTH_NORTH_IN_ENU
    else:
        leveled_reference = leveled_vehicle_heading(body_orientation)
    return body_orientation.inv() * leveled_reference * reference_rotation
