# Python imports
import yaml
from pathlib import Path

# ROS imports 
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node

# MAVInsight imports
from models.vehicle import Vehicle

class VehicleTfPublisher(Node):
    """The TF Publisher for Vehicles described in {TODO: configuration file path}"""
    vehicles: list[Vehicle]

    def __init__(self):
        super().__init__("vehicle_tf_publisher")

        # get vehicle configs
        self.vehicles = []
        self.build_vehicles()

    def build_vehicles(self, build_list:list[str] = None):
        """Construct a set of vehicles and their sensors based on their descriptions found in /{ws root dir}/vehicles/{vehicle.yaml}.

        Parameters
            (optional) build_list   A list of strings specifying which vehicles to build for this viz session.
                                        - If included, only vehicles specified in build_list will be built
                                        - If excluded, ALL vehicles in the vehicle directory will be built
        """
        vehicle_dir = Path(get_package_share_directory("mavinsight"))
        if (not build_list) or (len(build_list) == 0):
            vehicles = [p for p in (vehicle_dir / 'vehicles').iterdir()]
        else:
            vehicles = [vehicle_dir / 'vehcilces' / v for v in build_list]

        for vehicle_path in vehicles:
            if vehicle_path.is_file():
                try:
                    with open(vehicle_path, 'r', encoding='utf-8') as v:
                        self.vehicles.append(Vehicle(yaml.safe_load(v)))
                except Exception as e:
                    self.get_logger().error(f"Error reading {vehicle_path}: {e}")
            else:
                self.get_logger().error(f"{vehicle_path} not recognized as file")
