#python imports
from pathlib import Path
import yaml

# ROS imports
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, logging
from launch_ros.actions import Node

package_name = "mavinsight"
namespace = "viz"
LOGGER = logging.get_logger('vehicle_launch_logger')

def generate_launch_description():
    ld = LaunchDescription()

    vehicle_dir = Path(get_package_share_directory(package_name)) / 'vehicles'
    initial_paths = [p for p in vehicle_dir.iterdir()]
    LOGGER.info(f"Initial paths: {[p.name for p in initial_paths]}")

    nodes = build_nodes(initial_paths)
    for node in nodes:
        ld.add_action(node)

    return ld

def build_nodes(paths : list[Path]) -> list[Node]:
    LOGGER.debug("Starting build")
    # initialize set of processed paths and output list
    processed = set()
    node_list = []

    while paths:
        # capture and error check next path
        config_path = paths.pop()
        LOGGER.info(f"Starting processing on {config_path}")
        assert isinstance(config_path, Path), f"Unrecognized build_nodes input type."
        if config_path in processed:
            LOGGER.error(f"Potential circular path detected in config files.\nConfig file: {config_path.as_posix()} is contained by a sub-member")
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
        if abs_path == None:
            LOGGER.error(f"Cannot find file: {config_path.as_posix()} in any MAVInsight config folder.\nSkipping...")
            continue
        LOGGER.debug(f"abs path acquired")

        with open(abs_path.as_posix(), 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        if type(config) is not dict:
            LOGGER.error(f"Error parsing file: {abs_path.as_posix()} as yaml. GraphMember configs must be yaml-encoded.")
            continue
        LOGGER.debug(f"file opened successfully")

        while len(config.keys()) == 1:
            config = config[next(iter(config))]
        LOGGER.debug(f"Base yaml acquired")
        try:
            if 'platform' in config.keys():
                ex = 'vehicle'
            elif 'sensor_type' in config.keys():
                ex = config['sensor_type']
            else:
                LOGGER.error(f"Cannot determine the type of file: {abs_path.as_posix()}\nSkipping ...")
                continue
        except KeyError as e:
            LOGGER.error(f"Error parsing config file: {abs_path.as_posix()}, {e}\nSkipping...")
            continue
        LOGGER.debug(f"File type identified")

        node_list.append(Node(
            package=package_name,
            executable=ex,
            name=abs_path.stem,
            namespace=namespace,
            parameters=[abs_path.as_posix()],
            output='screen'
        ))

        LOGGER.info(f"Add new config files {config.get('sensors'), []}")
        for sens in config.get('sensors', []):
            paths.append(Path(sens))

    return node_list

def resolve_config_file(path:Path) -> Path | None:
    if path.is_absolute():
        return path

    ws_root = Path(get_package_share_directory(package_name))
    v_path = ws_root / 'vehicles' / path
    v_path = v_path if v_path.is_file() else None
    s_path = ws_root / 'sensors' / path
    s_path = s_path if s_path.is_file() else None

    if v_path and s_path:
        raise FileExistsError
    else:
        return v_path or s_path
