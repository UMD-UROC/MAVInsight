# Python imports
from pathlib import Path

# ROS imports
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node

# MAVInsight imports
from models.vehicle import Vehicle

class VehicleTfPublisher(Node):
    """The TF Publisher for Vehicles described in mavinsight/vehicles/*"""
    vehicles: list[Vehicle]

    def __init__(self):
        super().__init__("vehicle_tf_publisher")

        # get vehicle configs
        self.vehicles = []
        self.build_vehicles()

        self.create_timer(2.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("\n" + "\n".join([str(v) for v in self.vehicles]))

    def build_vehicles(self, build_list:list[str] = None):
        """Construct a set of vehicles and their sensors based on their descriptions found in /{ws root dir}/vehicles/{vehicle.yaml}.

        Parameters
            (optional) build_list   A list of strings specifying which vehicles to build for this viz session.
                                        - If included, only vehicles specified in build_list will be built
                                        - If excluded, ALL vehicles in the vehicle directory will be built
        """
        vehicles = []
        vehicle_dir = Path(get_package_share_directory("mavinsight"))
        if (not build_list) or (len(build_list) == 0):
            vehicles = [path.as_posix() for path in (vehicle_dir / 'vehicles').iterdir()]
        else:
            vehicles = build_list

        for vehicle_path in vehicles:
            self.get_logger().info(f"Attempting to build Vehicle from: {vehicle_path}")
            try:
                self.vehicles.append(vehicle_factory(vehicle_path))
            except (FileNotFoundError, PermissionError, IsADirectoryError, OSError, TypeError) as e:
                self.get_logger().error(f"Error reading file {vehicle_path}: {e}")
            except (ValueError) as e:
                self.get_logger().warn(f"Error while building vehicle from file: {vehicle_path}: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = VehicleTfPublisher()

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
