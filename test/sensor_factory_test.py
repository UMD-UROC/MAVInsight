import os
import unittest
from models.sensor import Camera, Gimbal, Rangefinder, sensor_factory


class TestSensorFactory(unittest.TestCase):
    def setUp(self):
        self.share_dir = os.path.dirname(__file__)

        self.sensor_dir = os.path.join(self.share_dir, "sensors")

        self.cm_dir = os.path.join(self.sensor_dir, "cameras")
        self.gb_dir = os.path.join(self.sensor_dir, "gimbals")
        self.rf_dir = os.path.join(self.sensor_dir, "rangefinders")

        self.full_camera: Camera = sensor_factory(
            os.path.join(self.cm_dir, "full_camera.yaml")
        )
        self.full_gimbal: Gimbal = sensor_factory(
            os.path.join(self.gb_dir, "full_gimbal.yaml")
        )
        self.full_rangefinder: Rangefinder = sensor_factory(
            os.path.join(self.rf_dir, "full_rangefinder.yaml")
        )

    def test_sensor_factory(self):
        """sensor factory correctly makes types from yamls"""
        self.assertRaises(FileNotFoundError, sensor_factory, "not_a_file")
        self.assertRaises(
            ValueError,
            sensor_factory,
            os.path.join(self.sensor_dir, "no_sensortype.yaml"),
        )
        self.assertRaises(
            ValueError,
            sensor_factory,
            os.path.join(self.sensor_dir, "bad_sensortype.yaml"),
        )

        self.assertIsInstance(self.full_camera, Camera)
        self.assertIsInstance(self.full_gimbal, Gimbal)
        self.assertIsInstance(self.full_rangefinder, Rangefinder)
