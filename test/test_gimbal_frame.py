"""Tests for MAVLink gimbal yaw-frame selection."""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R

from models.gimbal_frame import (FLAGS_PITCH_LOCK, FLAGS_ROLL_LOCK,
                                 FLAGS_YAW_IN_EARTH_FRAME,
                                 FLAGS_YAW_IN_VEHICLE_FRAME, FLAGS_YAW_LOCK,
                                 gimbal_reference_from_body,
                                 yaw_is_earth_referenced)


LEVEL_LOCKS = FLAGS_ROLL_LOCK | FLAGS_PITCH_LOCK


class TestGimbalFrame(unittest.TestCase):
    """Verify explicit and legacy frame reports compose only once."""

    def test_explicit_frame_flags_override_legacy_yaw_lock(self):
        """Explicit frame flags take priority over the compatibility bit."""
        self.assertTrue(yaw_is_earth_referenced(
            LEVEL_LOCKS | FLAGS_YAW_IN_EARTH_FRAME))
        self.assertFalse(yaw_is_earth_referenced(
            LEVEL_LOCKS | FLAGS_YAW_LOCK | FLAGS_YAW_IN_VEHICLE_FRAME))

    def test_legacy_yaw_lock_remains_the_fallback(self):
        """Old devices still select the frame through YAW_LOCK."""
        self.assertTrue(yaw_is_earth_referenced(LEVEL_LOCKS | FLAGS_YAW_LOCK))
        self.assertFalse(yaw_is_earth_referenced(LEVEL_LOCKS))

    def test_earth_frame_report_does_not_add_vehicle_yaw(self):
        """An earth-frame quaternion stays independent of vehicle yaw."""
        body = R.from_euler("xyz", [4.0, -7.0, 31.0], degrees=True)
        reference = gimbal_reference_from_body(
            body, LEVEL_LOCKS | FLAGS_YAW_IN_EARTH_FRAME)
        world_heading = (body * reference).as_euler("xyz", degrees=True)[2]
        self.assertAlmostEqual(world_heading, 90.0)

    def test_vehicle_frame_report_keeps_vehicle_heading(self):
        """A forward body-frame quaternion follows vehicle heading once."""
        body = R.from_euler("xyz", [4.0, -7.0, 31.0], degrees=True)
        reference = gimbal_reference_from_body(
            body, LEVEL_LOCKS | FLAGS_YAW_IN_VEHICLE_FRAME)
        world_attitude = (body * reference).as_euler("xyz", degrees=True)
        self.assertAlmostEqual(world_attitude[0], 0.0)
        self.assertAlmostEqual(world_attitude[1], 0.0)
        self.assertAlmostEqual(world_attitude[2], 31.0)

    def test_airframe_can_override_a_misreported_vehicle_yaw_frame(self):
        """V3's world-reported attitude gets an earth-fixed +90 degree reference."""
        body = R.from_euler("xyz", [4.0, -7.0, 31.0], degrees=True)
        reference = gimbal_reference_from_body(
            body,
            LEVEL_LOCKS | FLAGS_YAW_IN_VEHICLE_FRAME,
            yaw_is_earth_referenced_override=True)
        self.assertTrue(np.allclose(
            (body * reference).as_matrix(),
            R.from_euler("z", 90.0, degrees=True).as_matrix()))


if __name__ == "__main__":
    unittest.main()
