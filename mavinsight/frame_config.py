"""Resolved MAVInsight frame-tree configurations shared by launch and replay."""
import math
from pathlib import Path
from string import Template

import yaml
from ament_index_python import get_package_share_directory
from models.gimbal_frame import resolve_gimbal_pose_source

PACKAGE = 'mavinsight'
VEHICLE_CONFIG = 'chimera.yaml'
SIM_VEHICLE_CONFIG = 'sim_vehicle.yaml'


def localization_dynamic_edges(members, frames):
    """Required dynamic edges on the configured localization and anchor paths.

    Include the rangefinder even though ordinary ground localization can work
    without it. Do not include auxiliary telemetry/calibration/altitude planes.
    """
    parents = {}
    dynamic = set()

    def add(parent, child, *, is_static=False):
        if child in parents and parents[child] != parent:
            raise ValueError(f'Multiple parents for localization frame {child}')
        parents[child] = parent
        if not is_static:
            dynamic.add((parent, child))

    for _, config in members:
        frame, parent = config['frame_name'], config['parent_frame']
        if 'ekf_origin_frame' in config:
            if config.get('publish_fiducial_edge', True):
                add(config['fiducial_frame'], config['uncorrected_home_frame_name'])
                add(config['uncorrected_home_frame_name'], config['home_frame_name'])
            add(config['home_frame_name'], config['ekf_origin_frame'])
            add(parent, frame)
            add(config['gimbal_offset_frame'], config['gimbal_ref_frame'])
            continue
        if config.get('offset'):
            add(parent, frame + '_offset', is_static=True)
            parent = frame + '_offset'
        if config.get('sensor_type') == 'gimbal':
            add(frame + '_ref', frame)
            if config.get('encoder_enabled'):
                add(parent, frame + '_encoder')
        elif config.get('sensor_type') == 'rangefinder':
            add(parent, frame)
        elif config.get('sensor_type') == 'camera':
            add(parent, frame + '_optical', is_static=True)

    root = frames['localization_fiducial_frame']
    required = set()
    for key in ('localization_origin_frame', 'localization_uncorrected_origin_frame',
                'drone_frame', 'camera_frame', 'gimbal_frame', 'rangefinder_frame'):
        child = frames[key]
        visited = set()
        while child != root:
            if child in visited or child not in parents:
                raise ValueError(f'No configured localization TF path from {root} to {frames[key]}')
            visited.add(child)
            parent = parents[child]
            if (parent, child) in dynamic:
                required.add((parent, child))
            child = parent
    return sorted(required)


def dynamic_frame_owners(members, namespace='/viz'):
    """Map possible dynamic child frames to their configured publishing nodes.

    Optional edges become monitored only once observed. Camera mounts and
    optical frames are static and therefore excluded.
    """
    owners = {}
    for name, config in members:
        frame = config['frame_name']
        sensor_type = config.get('sensor_type')
        children = []
        if sensor_type == 'gimbal':
            children = [frame, frame + '_uncorrected']
            if config.get('encoder_enabled'):
                children += [frame + suffix for suffix in (
                    '_reported_full', '_encoder', '_encoder_uncorrected')]
        elif sensor_type == 'rangefinder':
            children = [frame]
        elif 'ekf_origin_frame' in config:  # Vehicle owns body and reference edges.
            children = [frame, frame + '_alt_plane', config['ekf_origin_frame'],
                        config['gimbal_ref_frame']]
            if config.get('publish_fiducial_edge', True):
                children += [config['home_frame_name'], config['uncorrected_home_frame_name']]
        for child in children:
            if child in owners:
                raise ValueError(f'Multiple MAVInsight nodes own dynamic frame {child}')
            owners[child] = namespace.rstrip('/') + '/' + name
    return owners


def frame_configs(number, model, *, camera='rgb', sim=False, bench=False,
                  publish_fiducial_edge=True, fiducial_lla='', camera_rotation=None,
                  localization_reference_topic='', gimbal_encoder=False,
                  gimbal_pose_source='auto'):
    """Build the same named TF-node configurations for live launch and bag replay.

    Replay writes these dictionaries to parameter files with use_sim_time set
    before starting each process. It does not maintain a second frame tree.
    """
    if camera not in ('rgb', 'thermal') or model not in ('v2', 'v3'):
        raise ValueError('invalid camera or airframe model')
    pose_source = resolve_gimbal_pose_source(gimbal_pose_source, gimbal_encoder)
    camera_rotation = [0.0, 0.0, 0.0] if camera_rotation is None else camera_rotation
    if len(camera_rotation) != 3 or not all(math.isfinite(v) for v in camera_rotation):
        raise ValueError('camera_mount_rotation_deg requires three finite Euler xyz angles')
    resources = Path(get_package_share_directory(PACKAGE)) / 'package_resources'
    nodes = []
    built = set()
    pending = [SIM_VEHICLE_CONFIG if sim else VEHICLE_CONFIG]
    gimbal_has_yaw_axis = None

    while pending:
        file_name = pending.pop(0)
        if file_name in built:
            raise RuntimeError(f'{file_name} is reached twice. The tree has a cycle.')
        built.add(file_name)

        config = load(resources / file_name, number)
        if config.get('parent_frame') == f'uas{number}_gimbal_frame' and pose_source == 'encoder':
            # Preserve the camera/optical/laser leaf names: all consumers use
            # the selected pose without building a second sensor subtree.
            config['parent_frame'] += '_encoder'
        if file_name in ('gimbal_rgb_camera.yaml', 'sim_gimbal_rgb_camera.yaml'):
            # The active camera retains the rgb frame names for downstream
            # localization and Foxglove. Its intrinsics and mount must both
            # correspond to camera:=rgb (v2 pilot) or camera:=thermal.
            config['display_name'] = config['display_name'].replace(
                'RGB', 'Thermal' if camera == 'thermal' else 'RGB')
            # The existing offset reader uses Euler xyz degrees. Preserve its
            # translation and apply the mounting correction once, in TF; all
            # measurement and visualization consumers then share this frame.
            config['camera_mount_rotation_deg'] = camera_rotation
            config['offset'] = config['offset'][:3] + camera_rotation
        if 'models' in config:
            config['sensors'] = config.pop('models')[model]
        if 'gimbal_reference_by_model' in config:
            config.update(config.pop('gimbal_reference_by_model')[model])
        if 'gimbal_has_yaw_axis_by_model' in config:
            # The vehicle declaration is the sole model-capability source for
            # both halves of the gimbal TF chain.
            gimbal_has_yaw_axis = bool(config.pop(
                'gimbal_has_yaw_axis_by_model')[model])
            config['gimbal.has_yaw_axis'] = gimbal_has_yaw_axis
        if config.get('sensor_type') == 'gimbal':
            if gimbal_has_yaw_axis is None:
                raise RuntimeError('gimbal sensor was loaded before its vehicle capability')
            # V2 is pitch/roll-only, so its reported yaw is non-physical. V3
            # retains the existing full reported-attitude path.
            config['ignore_reported_yaw'] = not gimbal_has_yaw_axis
            config['encoder_enabled'] = bool(gimbal_encoder)
            config['encoder_has_yaw_axis'] = gimbal_has_yaw_axis
            config['encoder_topic'] = f'/uas{number}/gimbal_encoder/joint_states'
        if bench and file_name == (SIM_VEHICLE_CONFIG if sim else VEHICLE_CONFIG):
            config['bench_base_altitude'] = 20.0
        if file_name == (SIM_VEHICLE_CONFIG if sim else VEHICLE_CONFIG):
            config['publish_fiducial_edge'] = publish_fiducial_edge
            config['localization_reference_topic'] = localization_reference_topic
            if fiducial_lla:
                config['fiducial_lla'] = [float(value) for value in fiducial_lla.split(',')]

        # Prefixes separate vehicles in the ground station's shared domain.
        nodes.append((f'd{number}_{Path(file_name).stem}', config))
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
