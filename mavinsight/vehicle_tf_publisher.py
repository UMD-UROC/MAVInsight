from functools import partial
from pathlib import Path

import rclpy
from ament_index_python import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Gimbal, Rangefinder

from models.sensor_types import SensorTypes
from models.vehicle import Vehicle, vehicle_factory


class VehicleTFPublisher(Node):
    """The TF Publisher for Vehicles described in mavinsight/vehicles/*"""

    vehicles: list[Vehicle]

    def __init__(self):
        super().__init__("vehicle_tf_publisher")

        self.vehicles = []
        self.build_vehicles()

        if len(self.vehicles) == 0:
            self.get_logger().error("No vehicles found. Exiting...")
            rclpy.shutdown()
            return

        # Subscribe to odometry topics for each vehicle
        # Use partial to bind the vehicle instance to the callback for proper routing
        for vehicle in self.vehicles:
            callback = partial(self.location_callback, vehicle=vehicle)
            self.create_subscription(Odometry, vehicle.location_topic, callback, 1)

        # Helper functions to recursively collect all sensors, including nested ones
        # (e.g., cameras mounted on gimbals)
        def get_all_sensors(vehicle: Vehicle):
            """Recursively get all sensors from a vehicle and its nested sensors."""
            all_sensors = []
            for sensor in vehicle.sensors:
                all_sensors.append((vehicle, sensor))
                all_sensors.extend(get_nested_sensors(vehicle, sensor))
            return all_sensors

        def get_nested_sensors(parent_vehicle: Vehicle, parent_sensor):
            """Recursively get all nested sensors."""
            nested = []
            for nested_sensor in parent_sensor.sensors:
                nested.append((parent_vehicle, nested_sensor))
                nested.extend(get_nested_sensors(parent_vehicle, nested_sensor))
            return nested

        # Subscribe to sensor topics for all sensors across all vehicles
        # Flatten the list comprehension to get (vehicle, sensor) pairs from all vehicles
        for vehicle, sensor in [s for v in self.vehicles for s in get_all_sensors(v)]:
            if sensor.sensor_type == SensorTypes.CAMERA:
                callback = partial(
                    self.cam_info_callback, vehicle=vehicle, sensor=sensor
                )
                self.create_subscription(
                    CameraInfo, sensor.cam_info_topic, callback, 10
                )
            elif sensor.sensor_type == SensorTypes.GIMBAL:
                callback = partial(
                    self.orientation_callback, vehicle=vehicle, sensor=sensor
                )
                self.create_subscription(Gimbal, sensor.orientation_topic, callback, 10)
            elif sensor.sensor_type == SensorTypes.RANGEFINDER:
                callback = partial(self.range_callback, vehicle=vehicle, sensor=sensor)
                self.create_subscription(Rangefinder, sensor.range_topic, callback, 10)

        # Timer for periodic tasks (e.g., publishing static transforms)
        self.create_timer(2.5, self.timer_callback)

    def location_callback(self, msg: Odometry, vehicle: Vehicle):
        """Callback for vehicle location/odometry messages.

        Parameters
        ----------
        msg : Odometry
            The odometry message from the vehicle's location topic
        vehicle : Vehicle
            The vehicle instance that this message belongs to
        """
        pass

    def cam_info_callback(self, msg: CameraInfo, vehicle: Vehicle, sensor):
        """Callback for camera info messages.

        Parameters
        ----------
        msg : CameraInfo
            The camera info message from the sensor's cam_info_topic
        vehicle : Vehicle
            The vehicle instance that owns this sensor
        sensor : Sensor
            The sensor instance (Camera) that this message belongs to
        """
        pass

    def orientation_callback(self, msg: Gimbal, vehicle: Vehicle, sensor):
        """Callback for gimbal orientation messages.

        Parameters
        ----------
        msg : Gimbal
            The gimbal orientation message from the sensor's orientation_topic
        vehicle : Vehicle
            The vehicle instance that owns this sensor
        sensor : Sensor
            The sensor instance (Gimbal) that this message belongs to
        """
        pass

    def range_callback(self, msg: Rangefinder, vehicle: Vehicle, sensor):
        """Callback for rangefinder messages.

        Parameters
        ----------
        msg : Rangefinder
            The rangefinder message from the sensor's range_topic
        vehicle : Vehicle
            The vehicle instance that owns this sensor
        sensor : Sensor
            The sensor instance (Rangefinder) that this message belongs to
        """
        pass

    def timer_callback(self):
        self.get_logger().info("\n" + "\n".join([str(v) for v in self.vehicles]))

    def build_vehicles(self, build_list: list[str] = None):
        """

        Construct a set of vehicles and their sensors based on their descriptions found in /{ws root dir}/vehicles/{vehicle.yaml}.

        Parameters
            (optional) build_list   A list of strings specifying which vehicles to build for this viz session.
                                        - If included, only vehicles specified in build_list will be built
                                        - If excluded, ALL vehicles in the vehicle directory will be built
        """

        vehicles = []
        vehicle_dir = Path(get_package_share_directory("mavinsight"))

        # If no build_list provided, discover all vehicle files in the vehicles directory
        if (not build_list) or (len(build_list) == 0):
            vehicles = [
                path.as_posix() for path in (vehicle_dir / "vehicles").iterdir()
            ]
        else:
            vehicles = build_list

        # Build each vehicle from its YAML configuration file
        for vehicle_path in vehicles:
            self.get_logger().info(f"Attempting to build Vehicle from: {vehicle_path}")
            try:
                self.vehicles.append(vehicle_factory(vehicle_path))
            except (
                FileNotFoundError,
                PermissionError,
                IsADirectoryError,
                OSError,
                TypeError,
            ) as e:
                # File system errors - log as error and continue
                self.get_logger().error(f"Error reading file {vehicle_path}: {e}")
            except ValueError as e:
                # Validation errors - log as warning and continue
                # (e.g., missing required fields, invalid YAML structure)
                self.get_logger().warn(
                    f"Error while building vehicle from file: {vehicle_path}: {e}"
                )


def main(args=None):
    rclpy.init(args=args)

    node = VehicleTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
