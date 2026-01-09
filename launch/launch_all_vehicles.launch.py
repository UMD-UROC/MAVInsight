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
initial_paths_overrides = ["chimera_d_4.yaml", "c2_c130_crash.yaml"]

def generate_launch_description():
    ld = LaunchDescription()

    share_dir = Path(get_package_share_directory(package_name))
    shared_resources = share_dir / "package_resources"

    global_config = shared_resources / 'global_node_config.yaml'

    # TODO change behavior for empty initial paths override
    initial_paths = [(shared_resources) / p for p in initial_paths_overrides]

    LOGGER.info(f"Initial paths: {[p.name for p in initial_paths]}")

    nodes = build_nodes(initial_paths, global_config)
    for node in nodes:
        ld.add_action(node)

    return ld

def build_nodes(paths: list[Path], global_config: Path) -> list[Node]:
    LOGGER.debug("Starting build")
    # initialize set of processed paths and output list
    processed = set()
    node_list = []

    while paths:
        # capture and error check next path
        config_path = paths.pop()
        LOGGER.info(f"Starting processing on {config_path.as_posix()}")
        assert isinstance(config_path, Path), f"Unrecognized build_nodes input type."
        if config_path.suffix != ".yaml":
            LOGGER.error(f"Non-yaml config file detected: {config_path.as_posix()}. GraphMember configs must be yaml-encoded.\nSkipping...")
            continue
        if config_path in processed:
            LOGGER.error(f"Potential circular path detected in config files.\nConfig file: {config_path.as_posix()} is contained by a sub-member.\nSkipping...")
            continue
        LOGGER.debug(f"non-circular path")
        # path is checkable, add to processed list
        processed.add(config_path)

        # resolve filename to absolute path in either sensor config or vehicle config
        try:
            abs_path = resolve_config_file(config_path)
        except FileExistsError:
            LOGGER.error(f"Duplicate filenames in Vehicle + Sensor dirs for file: {config_path.as_posix()}.\nSkipping...")
            continue
        if abs_path is None:
            LOGGER.error(f"Cannot find file: {config_path.as_posix()} in any MAVInsight config folder.\nSkipping...")
            continue
        LOGGER.debug(f"abs path acquired")

        # open file and confirm yaml encoding
        with open(abs_path.as_posix(), "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        if type(config) is not dict:
            LOGGER.error(f"Error parsing file: {abs_path.as_posix()} as yaml. GraphMember configs must be yaml-encoded.\nSkipping...")
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
            LOGGER.error(f"Config file: {abs_path.as_posix()} contains no executable param.\nSkipping...")
            continue
        LOGGER.debug(f"File type identified")

        # create Node action for launch description
        node_list.append(Node(
            package=package_name,
            executable=ex,
            name=abs_path.stem,
            namespace=namespace,
            parameters=[global_config.as_posix(), abs_path.as_posix()],
            output="screen",
        ))

        # add sub-members to list of nodes to be built
        LOGGER.info(f"Adding new config files: {config.get('sensors', [])}")
        for sens in config.get("sensors", []):
            paths.append(Path(sens))

    return node_list

def resolve_config_file(path: Path) -> Path | None:
    if path.is_absolute():
        return path

    package_configs = Path(get_package_share_directory(package_name)) / "package_resources"
    resolved_path = package_configs / path
    if not resolved_path.is_file():
        raise FileNotFoundError(f"Could not find configs for: {path} in mavinsight configs folder.")
    return resolved_path
