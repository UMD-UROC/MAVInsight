import os
import unittest
from models.platforms import Platforms
from models.sensor import Camera, Gimbal, Rangefinder, SensorTypes
from models.vehicle import Vehicle, vehicle_factory


class TestVehicle(unittest.TestCase):
    def setUp(self):
        self.share_dir = os.path.dirname(__file__)

        self.vehicle_dir = os.path.join(self.share_dir, "vehicles")
        self.bad_v_dir = os.path.join(self.vehicle_dir, "bad_vehicles")

        self.full_vehicle: Vehicle = vehicle_factory(
            os.path.join(self.vehicle_dir, "full_vehicle.yaml")
        )
        self.minimum_vehicle: Vehicle = vehicle_factory(
            os.path.join(self.vehicle_dir, "minimum_vehicle.yaml")
        )

    def test_vehicle_factory(self):
        """vehicle factory correctly makes or doesnt make gimbal"""
        self.assertIsInstance(self.full_vehicle, Vehicle)
        self.assertRaises(FileNotFoundError, vehicle_factory, "not_a_file")

    def test_vehicle_from_yaml(self):
        """vehicle creation from yaml"""
        self.assertRaises(
            ValueError, vehicle_factory, os.path.join(self.bad_v_dir, "v_no_name.yaml")
        )
        self.assertRaises(
            ValueError, vehicle_factory, os.path.join(self.bad_v_dir, "v_no_frame.yaml")
        )
        self.assertRaises(
            ValueError, vehicle_factory, os.path.join(self.bad_v_dir, "v_no_topic.yaml")
        )
        self.assertRaises(
            ValueError,
            vehicle_factory,
            os.path.join(self.bad_v_dir, "v_no_platform.yaml"),
        )

        # no parent case (guarded)
        vp = vehicle_factory(os.path.join(self.bad_v_dir, "v_no_parent.yaml"))
        self.assertEqual(vp.parent_frame, "map")

        # no sensors case (guarded)
        vns = vehicle_factory(os.path.join(self.bad_v_dir, "v_no_sensors.yaml"))
        self.assertIsInstance(vns.sensors, list)
        self.assertEqual(len(vns.sensors), 0)

        # empty sensors case (guarded)
        ves = vehicle_factory(os.path.join(self.bad_v_dir, "v_empty_sensors.yaml"))
        self.assertIsInstance(ves.sensors, list)
        self.assertEqual(len(ves.sensors), 0)

        # minimum vehicle case (all guarded params removed)
        vm = vehicle_factory(os.path.join(self.vehicle_dir, "minimum_vehicle.yaml"))
        self.assertEqual(vm.name, "Minimum Vehicle")
        self.assertEqual(vm.frame_name, "minimum_vehicle_base_link")
        self.assertEqual(vm.location_topic, "/minimum/vehicle/fix")
        self.assertEqual(vm.parent_frame, "map")
        self.assertEqual(vm.platform, Platforms.QUAD_COPTER)
        self.assertIsInstance(vm.sensors, list)
        self.assertEqual(len(vm.sensors), 0)

        # full vehicle checking
        self.assertEqual(self.full_vehicle.name, "Full Vehicle")
        self.assertEqual(self.full_vehicle.frame_name, "full_vehicle_base_link")
        self.assertEqual(self.full_vehicle.location_topic, "/full/vehicle/fix")
        self.assertEqual(self.full_vehicle.parent_frame, "full_vehicle_parent")
        self.assertEqual(self.full_vehicle.platform, Platforms.QUAD_COPTER)
        self.assertEqual(len(self.full_vehicle.sensors), 2)

        sub_cam: Camera = self.full_vehicle.sensors[0]
        self.assertEqual(sub_cam.name, "No Offset Camera")
        self.assertEqual(sub_cam.frame_name, "no_offset_cam")
        self.assertEqual(sub_cam.cam_info_topic, "/no/offset/cam")
        self.assertEqual(sub_cam.parent_frame, "link")
        self.assertEqual(sub_cam.sensor_type, SensorTypes.CAMERA)
        self.assertEqual(len(sub_cam.offset), 3)
        self.assertEqual(sub_cam.offset[0], 0.0)
        self.assertEqual(sub_cam.offset[1], 0.0)
        self.assertEqual(sub_cam.offset[2], 0.0)

        sub_gimbal: Gimbal = self.full_vehicle.sensors[1]
        self.assertEqual(sub_gimbal.name, "Full Gimbal")
        self.assertEqual(sub_gimbal.frame_name, "full_gimbal")
        self.assertEqual(sub_gimbal.orientation_topic, "/test/gimbal")
        self.assertEqual(sub_gimbal.parent_frame, "link")
        self.assertEqual(sub_gimbal.sensor_type, SensorTypes.GIMBAL)
        self.assertEqual(len(sub_gimbal.offset), 3)
        self.assertEqual(sub_gimbal.offset[0], 0.01)
        self.assertEqual(sub_gimbal.offset[1], 0.002)
        self.assertEqual(sub_gimbal.offset[2], 0.0003)

        g_cam: Camera = self.full_vehicle.sensors[1].sensors[0]
        self.assertEqual(g_cam.name, "Full test cam")
        self.assertEqual(g_cam.frame_name, "test_cam")
        self.assertEqual(len(g_cam.offset), 3)
        self.assertEqual(g_cam.offset[0], 9.0)
        self.assertEqual(g_cam.offset[1], 8.0)
        self.assertEqual(g_cam.offset[2], 7.6)
        self.assertEqual(g_cam.cam_info_topic, "/test/cam")
        self.assertEqual(g_cam.parent_frame, "link")
        self.assertEqual(g_cam.sensor_type, SensorTypes.CAMERA)

        g_rf: Rangefinder = self.full_vehicle.sensors[1].sensors[1]
        self.assertEqual(g_rf.name, "test full rangefinder")
        self.assertEqual(g_rf.frame_name, "test_rangefinder")
        self.assertEqual(len(g_rf.offset), 3)
        self.assertEqual(g_rf.offset[0], 1.0)
        self.assertEqual(g_rf.offset[1], 2.0)
        self.assertEqual(g_rf.offset[2], 3.0)
        self.assertEqual(g_rf.range_topic, "/test/rangefinder")
        self.assertEqual(g_rf.parent_frame, "link")
        self.assertEqual(g_rf.sensor_type, SensorTypes.RANGEFINDER)
