#python imports
from pathlib import Path

# ROS imports
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, logging
from launch_ros.actions import Node

package_name = "mavinsight"
namespace = "viz"

def generate_launch_description():
    ld = LaunchDescription()
    LOGGER = logging.get_logger('vehicle_launch_logger')

    vehicle_dir = Path(get_package_share_directory(package_name)) / 'vehicles'
    for path in vehicle_dir.iterdir():
        LOGGER.info(f"Attempting to build Vehicle Node from: {path.name}")
        if path.suffix != ".yaml":
            LOGGER.error(f"Vehicle param file {path.as_posix()} is not yaml-encoded!")
            continue
        ld.add_action(Node(
            package=package_name,
            executable='vehicle',
            name=path.stem,
            namespace=namespace,
            parameters=[path.as_posix()]
        ))

    return ld