#python imports
from pathlib import Path

# ROS imports
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, logging
from launch_ros.actions import Node

package_name = "mavinsight"

def generate_launch_description():
    ld = LaunchDescription()
    LOGGER = logging.get_logger('vehicle_launch_logger')

    p = Path(get_package_share_directory(package_name))
    vehicles = [path.as_posix() for path in (p / 'vehicles').iterdir()]

    for vehicle_path in vehicles:
        LOGGER.info(f"Attempting to build Vehicle Node from: {vehicle_path}")


    return ld