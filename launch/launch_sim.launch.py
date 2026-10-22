"""The frame tree for one vehicle.

One file for every vehicle and both airframes. The configuration files hold
${N} where the UAS number belongs; this launch substitutes it and hands each
node the result, so a model is described once and instantiated per vehicle.

    ros2 launch mavinsight launch_sim.launch.py uas:=4 model:=v2

See px4-sim-stack/docs/uas-contract.md section 5.
"""

import math
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mavinsight.frame_config import frame_configs, load, as_floats

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
        DeclareLaunchArgument(
            'gimbal_encoder', default_value='false', choices=['true', 'false'],
            description='Publish measured encoder and full reported comparison frames.'),
        DeclareLaunchArgument(
            'gimbal_pose_source', default_value='auto', choices=['auto', 'encoder', 'fused'],
            description='Camera/laser pose: encoders when available, or legacy fused attitude.'),
        DeclareLaunchArgument(
            'bench', default_value='false', choices=['true', 'false'],
            description='Place the bench vehicle 20 m above its home frame.'),
        DeclareLaunchArgument(
            'publish_fiducial_edge', default_value='true', choices=['true', 'false'],
            description='Publish this vehicle\'s fiducial -> raw home -> corrected home TF '
                        'edges. Each vehicle owns its complete reference chain.'),
        DeclareLaunchArgument('localization_reference_topic', default_value='',
                              description='Onboard canonical reference for ground reconstruction.'),
        DeclareLaunchArgument('fiducial_lla', default_value='',
                              description='Known fiducial LLA as lat,lon,alt.'),
        DeclareLaunchArgument(
            'camera', default_value='rgb', choices=['rgb', 'thermal'],
            description='Selected gimbal camera: RGB/pilot by day or thermal by night.'),
        DeclareLaunchArgument(
            'camera_mount_rotation_deg', default_value='0.0,0.0,0.0',
            description='Selected gimbal camera mounting Euler xyz angles in degrees '
                        '(FLU axes), as roll,pitch,yaw. Keep CameraInfo R identity.'),
        OpaqueFunction(function=frame_tree),
    ])


def frame_tree(context, *args, **kwargs):
    number = int(LaunchConfiguration('uas').perform(context))
    model = LaunchConfiguration('model').perform(context)
    camera = LaunchConfiguration('camera').perform(context)
    sim = LaunchConfiguration('sim').perform(context) == 'true'
    bench = LaunchConfiguration('bench').perform(context) == 'true'
    encoder_enabled = LaunchConfiguration('gimbal_encoder').perform(context) == 'true'
    publish_fiducial_edge = LaunchConfiguration('publish_fiducial_edge').perform(context) == 'true'
    fiducial_lla = LaunchConfiguration('fiducial_lla').perform(context)
    reference_topic = LaunchConfiguration('localization_reference_topic').perform(context)
    camera_rotation = [float(value) for value in LaunchConfiguration(
        'camera_mount_rotation_deg').perform(context).split(',')]
    if len(camera_rotation) != 3 or not all(math.isfinite(value) for value in camera_rotation):
        raise ValueError('camera_mount_rotation_deg requires three finite Euler xyz angles')

    resources = Path(get_package_share_directory(PACKAGE)) / 'package_resources'
    global_config = str(resources / 'global_node_config.yaml')
    return [Node(
        package=PACKAGE, executable=config['executable'], name=name,
        namespace=NAMESPACE, parameters=[global_config, config], output='screen',
    ) for name, config in frame_configs(
        number, model, camera=camera, sim=sim, bench=bench,
        publish_fiducial_edge=publish_fiducial_edge, fiducial_lla=fiducial_lla,
        camera_rotation=camera_rotation, localization_reference_topic=reference_topic,
        gimbal_encoder=encoder_enabled,
        gimbal_pose_source=LaunchConfiguration('gimbal_pose_source').perform(context))]
