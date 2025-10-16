import os
import unittest
from ament_index_python import get_package_share_directory
from models.sensor import Camera, Gimbal, Rangefinder, sensor_factory
from models.sensor_types import SensorTypes

class TestFileIO(unittest.TestCase):
    def setUp(self):
        self.share_dir = os.path.dirname(__file__)

        self.vehicle_dir = os.path.join(self.share_dir, "vehicles")
        self.sensor_dir = os.path.join(self.share_dir, "sensors")

        self.cm_dir = os.path.join(self.sensor_dir, "cameras")
        self.gb_dir = os.path.join(self.sensor_dir, "gimbals")
        self.rf_dir = os.path.join(self.sensor_dir, "rangefinders")

        self.full_camera:Camera = sensor_factory(os.path.join(self.cm_dir, "full_camera.yaml"))
        self.full_gimbal:Gimbal = sensor_factory(os.path.join(self.gb_dir, "full_gimbal.yaml"))
        self.full_rangefinder:Rangefinder = sensor_factory(os.path.join(self.rf_dir, "full_rangefinder.yaml"))

    def test_sensor_factory(self):
        """Test that sensor factory can detect and make different sensor types from files"""
        self.assertRaises(FileNotFoundError, sensor_factory, "not_a_file")
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.sensor_dir, "no_sensortype.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.sensor_dir, "bad_sensortype.yaml"))

        self.assertIsInstance(self.full_camera, Camera)
        self.assertIsInstance(self.full_gimbal, Gimbal)
        self.assertIsInstance(self.full_rangefinder, Rangefinder)

    def test_rangefinder_from_yaml(self):
        """Test rangefinder creation logic"""
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_no_name.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_no_frame.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_no_parent.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_no_topic.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_no_type.yaml"))

        # offset cases
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_long_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_short_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_none_in_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.rf_dir, "rf_bad_offset.yaml"))

        rf0 = sensor_factory(os.path.join(self.rf_dir, "rf_no_offset.yaml"))
        self.assertEqual(rf0.offset[0], 0.0)
        self.assertEqual(rf0.offset[1], 0.0)
        self.assertEqual(rf0.offset[2], 0.0)

        self.assertEqual(self.full_rangefinder.name, "test full rangefinder")
        self.assertEqual(self.full_rangefinder.frame_name, "test_rangefinder")
        self.assertEqual(len(self.full_rangefinder.offset), 3)
        self.assertEqual(self.full_rangefinder.offset[0], 1.0)
        self.assertEqual(self.full_rangefinder.offset[1], 2.0)
        self.assertEqual(self.full_rangefinder.offset[2], 3.0)
        self.assertEqual(self.full_rangefinder.range_topic, "/test/rangefinder")
        self.assertEqual(self.full_rangefinder.parent_frame, "link")
        self.assertEqual(self.full_rangefinder.sensor_type, SensorTypes.RANGEFINDER)

    def test_camera_from_yaml(self):
        """Test camera creation logic"""
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

    def test_gimbal_from_yaml(self):
        """Test gimbal creation logic"""
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_no_name.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_no_frame.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_no_parent.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_no_topic.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_no_type.yaml"))

        # offset cases
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_long_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_short_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_none_in_offset.yaml"))
        self.assertRaises(ValueError, sensor_factory, os.path.join(self.gb_dir, "gb_bad_offset.yaml"))

        gb0 = sensor_factory(os.path.join(self.gb_dir, "gb_no_offset.yaml"))
        self.assertEqual(gb0.offset[0], 0.0)
        self.assertEqual(gb0.offset[1], 0.0)
        self.assertEqual(gb0.offset[2], 0.0)

        gbe = sensor_factory(os.path.join(self.gb_dir, "gb_empty_offset.yaml"))
        self.assertEqual(gbe.offset[0], 0.0)
        self.assertEqual(gbe.offset[1], 0.0)
        self.assertEqual(gbe.offset[2], 0.0)

        self.assertEqual(self.full_gimbal.name, "Full Gimbal")
        self.assertEqual(self.full_gimbal.frame_name, "full_gimbal")
        self.assertEqual(len(self.full_gimbal.offset), 3)
        self.assertEqual(self.full_gimbal.offset[0], 0.01)
        self.assertEqual(self.full_gimbal.offset[1], 0.002)
        self.assertEqual(self.full_gimbal.offset[2], 0.0003)
        self.assertEqual(self.full_gimbal.orientation_topic, "/test/gimbal")
        self.assertEqual(self.full_gimbal.parent_frame, "link")
        self.assertEqual(self.full_gimbal.sensor_type, SensorTypes.GIMBAL)

        sub_cam = self.full_gimbal.sensors[0]
        self.assertEqual(sub_cam.name, "Full test cam")
        self.assertEqual(sub_cam.frame_name, "test_cam")
        self.assertEqual(len(sub_cam.offset), 3)
        self.assertEqual(sub_cam.offset[0], 9.0)
        self.assertEqual(sub_cam.offset[1], 8.0)
        self.assertEqual(sub_cam.offset[2], 7.6)
        self.assertEqual(sub_cam.cam_info_topic, "/test/cam")
        self.assertEqual(sub_cam.parent_frame, "link")
        self.assertEqual(sub_cam.sensor_type, SensorTypes.CAMERA)

        sub_rf = self.full_gimbal.sensors[1]
        self.assertEqual(sub_rf.name, "test full rangefinder")
        self.assertEqual(sub_rf.frame_name, "test_rangefinder")
        self.assertEqual(len(sub_rf.offset), 3)
        self.assertEqual(sub_rf.offset[0], 1.0)
        self.assertEqual(sub_rf.offset[1], 2.0)
        self.assertEqual(sub_rf.offset[2], 3.0)
        self.assertEqual(sub_rf.range_topic, "/test/rangefinder")
        self.assertEqual(sub_rf.parent_frame, "link")
        self.assertEqual(sub_rf.sensor_type, SensorTypes.RANGEFINDER)
