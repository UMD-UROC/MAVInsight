import os
import unittest
from models.sensor import Camera, sensor_factory
from models.sensor_types import SensorTypes

class TestCamera(unittest.TestCase):
    def setUp(self):
        self.share_dir = os.path.dirname(__file__)

        self.sensor_dir = os.path.join(self.share_dir, "sensors")

        self.cm_dir = os.path.join(self.sensor_dir, "cameras")

        self.full_camera: Camera = sensor_factory(os.path.join(self.cm_dir, "full_camera.yaml"))

    def test_sensor_factory(self):
        """sensor factory correctly makes camera"""
        self.assertIsInstance(self.full_camera, Camera)

    def test_camera_from_yaml(self):
        """camera creation from yaml"""
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_no_name.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_no_frame.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_no_parent.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_no_topic.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_no_type.yaml"))

        # offset cases
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_long_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_short_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_none_in_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.cm_dir, "cm_bad_offset.yaml"))

        cm0 = sensor_factory(os.path.join(self.cm_dir, "cm_no_offset.yaml"))
        self.assertEqual(cm0.offset[0], 0.0)
        self.assertEqual(cm0.offset[1], 0.0)
        self.assertEqual(cm0.offset[2], 0.0)

        self.assertEqual(self.full_camera.name, "Full test cam")
        self.assertEqual(self.full_camera.frame_name, "test_cam")
        self.assertEqual(len(self.full_camera.offset), 3)
        self.assertEqual(self.full_camera.offset[0], 9.0)
        self.assertEqual(self.full_camera.offset[1], 8.0)
        self.assertEqual(self.full_camera.offset[2], 7.6)
        self.assertEqual(self.full_camera.cam_info_topic, "/test/cam")
        self.assertEqual(self.full_camera.parent_frame, "link")
        self.assertEqual(self.full_camera.sensor_type, SensorTypes.CAMERA)
