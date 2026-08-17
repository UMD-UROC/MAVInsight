"""Build a frame-tree launch from a specification file.

A launch spec names the vehicles to bring up. Each vehicle config names its
sensors, each sensor may name more, and every one of them becomes a node that
publishes its own link of the transform tree. The walk is breadth-first over
that graph and stops at anything it has already visited, so a config that
lists itself under a sub-member is reported rather than expanded forever.

Every per-vehicle launch file in this package is this function plus a
specification name, so the three of them cannot drift apart.
"""

from pathlib import Path

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, logging
from launch_ros.actions import Node

PACKAGE_NAME = "mavinsight"
NAMESPACE = "viz"

LOGGER = logging.get_logger("vehicle_launch_logger")
SHARED_RESOURCES = Path(get_package_share_directory(PACKAGE_NAME)) / "package_resources"
GLOBAL_CONFIG = SHARED_RESOURCES / "global_node_config.yaml"


def launch_description(specs_name: str) -> LaunchDescription:
    """The launch description for one spec file under package_resources."""
    ld = LaunchDescription()
    for node in build_nodes(extract_paths(SHARED_RESOURCES / specs_name)):
        ld.add_action(node)
    return ld


def build_nodes(paths: list) -> list:
    processed = set()
    node_list = []
    available_configs = [p.name for p in SHARED_RESOURCES.glob("*.yaml")]

    while paths:
        system_name = paths.pop()
        if not system_name.endswith(".yaml"):
            system_name += ".yaml"
        LOGGER.info(f"Starting processing on {system_name}")

        if system_name not in available_configs:
            LOGGER.error(f"Could not find {system_name} among config files. Skipping...")
            continue
        if system_name in processed:
            LOGGER.error(
                f"Potential circular path detected in config files.\n"
                f"Config file: {system_name} is contained by a sub-member.\nSkipping...")
            continue
        processed.add(system_name)

        path = SHARED_RESOURCES / system_name
        with open(path.as_posix(), "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        if type(config) is not dict:
            LOGGER.error(
                f"Error parsing file: {path.as_posix()} as yaml. GraphMember configs "
                f"must be yaml-encoded.\nSkipping...")
            continue

        # parse yaml down to the param layer (remove the layers of nesting above params)
        while len(config.keys()) == 1:
            config = config[next(iter(config))]

        try:
            executable = config["executable"]
        except KeyError:
            LOGGER.error(
                f"Config file: {path.as_posix()} contains no \"executable\" param.\n"
                f"Skipping...")
            continue

        node_list.append(Node(
            package=PACKAGE_NAME,
            executable=executable,
            name=path.stem,
            namespace=NAMESPACE,
            parameters=[GLOBAL_CONFIG.as_posix(), path.as_posix()],
            output="screen",
        ))

        sensors = config.get("sensors", [])
        if sensors:
            LOGGER.info(f"Adding new sensor files: {sensors}")
            paths.extend(sensors)

    return node_list


def extract_paths(path: Path) -> list:
    with open(path.as_posix(), "r", encoding="utf-8") as f:
        specs = yaml.safe_load(f)
    if type(specs) is not dict:
        LOGGER.error(f"Error parsing file: {path.as_posix()} as yaml. Could not load specs")
        return []
    return list(specs["vehicles"])
