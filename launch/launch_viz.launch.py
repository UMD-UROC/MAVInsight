# python imports
from pathlib import Path
import yaml

# ROS imports
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, logging
from launch_ros.actions import Node

package_name = "mavinsight"
namespace = "viz"
LOGGER = logging.get_logger("vehicle_launch_logger")
share_dir = Path(get_package_share_directory(package_name))
shared_resources = share_dir / "package_resources"
global_config = shared_resources / 'global_node_config.yaml'
launch_specs = shared_resources / 'launch_specs.yaml'

def generate_launch_description():
    ld = LaunchDescription()

    # new_tba_launcher = Node(
    #     package=package_name,
    #     executable='tba_viz',
    #     name='new_tba',
    #     namespace=namespace,
    #     parameters=[global_config.as_posix(), (shared_resources/'uas4_tba.yaml').as_posix()],
    #     output="screen",
    # )

    legacy_loczn_viz_launcher = Node(
        package=package_name,
        executable='tba_viz',
        name='uas4_tba',
        namespace=namespace,
        parameters=[global_config.as_posix(), (shared_resources/'uas4_tba.yaml').as_posix()],
        output="screen",
    )

    tf_loczn_viz_launcher = Node(
        package=package_name,
        executable='tba_viz',
        name='uas4_tf_loc_tba',
        namespace=namespace,
        parameters=[global_config.as_posix(), (shared_resources/'tf_loc_tba.yaml').as_posix()],
        output="screen",
    )

    ld.add_action(legacy_loczn_viz_launcher)
    ld.add_action(tf_loczn_viz_launcher)
    return ld
