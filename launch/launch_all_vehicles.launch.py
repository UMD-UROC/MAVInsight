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

    # TODO: make this command-line configurable?
    initial_systems = extract_paths(launch_specs)
    LOGGER.info(f"Initial systems: {initial_systems}")

    nodes = build_nodes(initial_systems, global_config)
    for node in nodes:
        ld.add_action(node)

    return ld

def build_nodes(paths: list[str], global_config: Path) -> list[Node]:
    LOGGER.debug("Starting build")
    # initialize set of processed paths and output list
    processed = set()
    node_list = []
    available_configs = [p.name for p in shared_resources.glob("*.yaml")]

    while paths:
        # capture and error check next path
        system_name = paths.pop()
        if not system_name.endswith(".yaml"): system_name += ".yaml"
        LOGGER.info(f"Starting processing on {system_name}")

        # search the shared space for a yaml with this name
        if system_name not in available_configs:
            LOGGER.error(f"Could not find {system_name} among config files. Skipping...")
            continue
        # check if we have already processed this file
        if system_name in processed:
            LOGGER.error(f"Potential circular path detected in config files.\nConfig file: {system_name} is contained by a sub-member.\nSkipping...")
            continue
        LOGGER.debug(f"non-circular path")
        # path is checkable, add to processed list
        processed.add(system_name)

        # resolve filename to absolute path in either sensor config or vehicle config
        path = shared_resources / system_name
        if not path.is_file():
            LOGGER.error(f"Could not resolve: \"{path}\" as Path.")
        LOGGER.debug(f"abs path acquired")

        # open file and confirm yaml encoding
        with open(path.as_posix(), "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        if type(config) is not dict:
            LOGGER.error(f"Error parsing file: {path.as_posix()} as yaml. GraphMember configs must be yaml-encoded.\nSkipping...")
            continue
        LOGGER.debug(f"file opened successfully")

        # parse yaml down to the param layer (remove the layers of nesting above params)
        while len(config.keys()) == 1:
            config = config[next(iter(config))]
        LOGGER.debug(f"Base yaml acquired")

        # select the correct executable for this config file
        try:
            ex = config['executable']
        except KeyError as e:
            LOGGER.error(f"Config file: {path.as_posix()} contains no \"executable\" param.\nSkipping...")
            continue
        LOGGER.debug(f"File type identified")

        # create Node action for launch description
        node_list.append(Node(
            package=package_name,
            executable=ex,
            name=path.stem,
            namespace=namespace,
            parameters=[global_config.as_posix(), path.as_posix()],
            output="screen",
        ))

        # add sub-members to list of nodes to be built
        sensors = config.get('sensors', [])
        if len(sensors) > 0:
            LOGGER.info(f"Adding new sensor files: {sensors}")
            for sens in config.get("sensors", []):
                paths.append(sens)

        # still burgeoning functionality. removing until we build back up to localization viz
        # vizs = config.get('viz', [])
        # if len(vizs) > 0:
        #     LOGGER.info(f"Adding new visualization files: {vizs}")
        #     for viz in config.get("viz", []):
        #         paths.append(Path(viz))

    return node_list

def extract_paths(path: Path) -> list:
    with open(path.as_posix(), "r", encoding="utf-8") as f:
        specs = yaml.safe_load(f)
    if type(specs) is not dict:
        LOGGER.error(LOGGER.error(f"Error parsing file: {path.as_posix()} as yaml. Could not load specs"))
        return []
    return specs['vehicles']
