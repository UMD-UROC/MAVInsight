import os
import unittest
from models.sensor import Rangefinder, sensor_factory
from models.sensor_types import SensorTypes

class TestRangefinder(unittest.TestCase):
    def setUp(self):
        self.share_dir = os.path.dirname(__file__)

        self.sensor_dir = os.path.join(self.share_dir, "sensors")

        self.rf_dir = os.path.join(self.sensor_dir, "rangefinders")

        self.full_rangefinder: Rangefinder = sensor_factory(os.path.join(self.rf_dir, "full_rangefinder.yaml"))

    def test_sensor_factory(self):
        """sensor factory correctly makes rangefinder"""
        self.assertIsInstance(self.full_rangefinder, Rangefinder)

    def test_rangefinder_from_yaml(self):
        """rangefinder creation from yaml"""
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
