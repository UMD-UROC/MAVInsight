"""The frame tree for one vehicle.

One file for every vehicle and both airframes. The configuration files hold
${N} where the UAS number belongs; this launch substitutes it and hands each
node the result, so a model is described once and instantiated per vehicle.

    ros2 launch mavinsight launch_sim.launch.py uas:=4 model:=v2

See px4-sim-stack/docs/uas-contract.md section 5.
"""

from pathlib import Path
from string import Template

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'mavinsight'
NAMESPACE = 'viz'
# The airframe the frame tree is built from. The aircraft's dimensions are in
# chimera.yaml; the simulator's clone shares its component makeup but not its
# size, so `sim:=true` reads sim_vehicle.yaml and the sim_* sensors beneath it.
VEHICLE_CONFIG = 'chimera.yaml'
SIM_VEHICLE_CONFIG = 'sim_vehicle.yaml'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uas',
            description='UAS number. It gives the d<N> and uas<N> frame '
                        'prefixes and the /uas<N> topic names.'),
        DeclareLaunchArgument(
            'model', choices=['v2', 'v3'],
            description='Airframe model. It gives the sensor set.'),
        DeclareLaunchArgument(
            'sim', default_value='false', choices=['true', 'false'],
            description='Build the tree from the simulated airframe\'s '
                        'dimensions rather than the aircraft\'s.'),
        OpaqueFunction(function=frame_tree),
    ])


def frame_tree(context, *args, **kwargs):
    number = int(LaunchConfiguration('uas').perform(context))
    model = LaunchConfiguration('model').perform(context)
    sim = LaunchConfiguration('sim').perform(context) == 'true'

    resources = Path(get_package_share_directory(PACKAGE)) / 'package_resources'
    global_config = str(resources / 'global_node_config.yaml')

    nodes = []
    built = set()
    pending = [SIM_VEHICLE_CONFIG if sim else VEHICLE_CONFIG]

    while pending:
        file_name = pending.pop(0)
        if file_name in built:
            raise RuntimeError(f'{file_name} is reached twice. The tree has a cycle.')
        built.add(file_name)

        config = load(resources / file_name, number)
        if 'models' in config:
            config['sensors'] = config.pop('models')[model]

        nodes.append(Node(
            package=PACKAGE,
            executable=config['executable'],
            # The prefix keeps two vehicles apart on the one domain the ground
            # station runs both of their trees on.
            name=f'd{number}_{Path(file_name).stem}',
            namespace=NAMESPACE,
            parameters=[global_config, config],
            output='screen',
        ))
        pending += config.get('sensors', [])

    return nodes


def load(path, number):
    """Read one member's configuration and fill in the UAS number."""
    if not path.is_file():
        raise RuntimeError(f'{path} is missing. A frame tree with a hole in it is worse '
                           f'than no frame tree, so this is fatal.')
    config = yaml.safe_load(Template(path.read_text(encoding='utf-8')).substitute(N=number))
    return {key: as_floats(value) for key, value in config.items()}


def as_floats(value):
    """Make a list of numbers a list of doubles.

    A YAML list of whole numbers arrives as an integer array, and every reader
    of a numeric list here asks for a double array, which then comes back empty.
    An offset lost that way builds no frame and takes a branch of the tree with
    it, silently.
    """
    if isinstance(value, list) and value and all(
            isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
        return [float(item) for item in value]
    return value
