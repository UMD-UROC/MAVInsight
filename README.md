# MAVInsight
The `mavinsight` package is a ROS2 Package configured to build a model of a Vehicle for later visualization in Foxglove.

## Package Structure
- `MAVInsight/` - This repo.
  - `mavinsight/` - Source directory for the Nodes of this repo.
  // TODO UPDATE
    - `vehicle_tf_publisher.py` - The Node that will publish transforms between frames based on the the relationships between `Vehicles` and `Sensors`.
      - `Vehicle` definitions are written in `.yaml` files found in `vehicles/`
      - `Sensor` definitions are written in `.yaml` files found in `sensors/`
      - At startup, this node will look for `Vehicle` definitions in the `vehicles/` directory. It will either build all vehicles in that directory, or only those specified by the optional argument: `build_list`.
  - `models/` - Directory for the classes that define the relations between different frames.
    - `graph_member.py` - Parent class for anything that can be published to the 3D panel of Foxglove
    - `vehicle.py` - File for classes that represent a vehicle for `Sensors` (i.e. CHIMERA A/D, any other drones, stationary cameras, etc.). Instances of this class should be constructed by `vehicle_tf_publisher.py` based on the configurations specified in `vehicles/`.
    - `sensor.py` - File for class that represent a sensor (i.e. rangefinders, cameras, gimbals, etc.). Instances of this class or its subclasses are constructed at runtime during the creation of a `Vehicle` or a parent `Sensor` based on the configurations specified in `sensors/`.
    - `vehicles.py` - Enum to capture supported vehicle types. Marginally useful.
    - `sensor_types.py` - Enum to capture supported sensor types. Marginally useful.
  - `sensors/` - Directory for the config files that define an instance of a `Sensor`.
  - `vehicles/` - Directory for the config files that define an instance of a `Vehicle`.

## Config File Specifications
To define a new `Vehicle` or `Sensor` for vizualization, you have to write a new `.yaml` file in either `vehicles/` or `sensors/` (respectively).
The general format of a configuration file should be:
```config.yaml
property: value
list_property:
 - list_value1
 - list_value2
empyt_list_property: []
```

## Selected gimbal camera mounting correction

`launch_sim.launch.py camera:=rgb|thermal camera_mount_rotation_deg:=roll,pitch,yaw`
uses degrees about the camera FLU axes, with Euler xyz composition. The
`umd_uas` onboard and offboard launches forward the selected per-vehicle camera
parameters. RGB means the v3 RGB stream or the v2 pilot stream; thermal means
the thermal stream. Day/night switching restarts the launch with that selection.

The selected camera uses the existing `uas<N>_rgb_offset` and
`uas<N>_rgb_optical` frame names. Its CameraInfo and mounting transform must
belong to the same camera. The mounting rotation is applied once on the static
`gimbal_frame -> rgb_offset` edge; CameraInfo R stays identity. Measurements,
fiducial correction, and image/3D visualization therefore share the same TF.

## Localization reference and PX4 home updates

Vehicle owns the complete reference chain. Application `uasN_home_uncorrected`
and `uasN_home_position` are stable ENU frames anchored at the first complete
home received by the onboard Vehicle process. They are **not PX4's current RTL
home**. The original MAVROS `home_position/home` message remains unchanged.
`home_position/fix` now describes the stable application anchor.

Every home message is composed atomically as geographic home minus local ENU
home. Only this composed EKF offset enters the application tree. Paired home
altitude/position changes therefore cancel before calibration, localization,
terrain or visualization can see separate moving terms. Genuine changes to the
composed reference remain visible. ENU translations use the existing site-scale
identity-aligned frame convention; this does not add estimator-reset compensation.

`/uasN/localization/reference` is reliable, transient-local current state;
`/uasN/localization/reference_events` is reliable event history for recording and
live consumers. Both carry schema-1 JSON in `std_msgs/String`: process generation,
activation nanoseconds, fixed ellipsoid-height anchor, composed raw EKF offset,
survey translation and frame names. Geographic consumers should use one immutable
snapshot at measurement time. Home and calibration state are sample-and-hold;
continuous body/sensor telemetry remains in TF. Historical requests preceding
available reference history must fail rather than use future state.

For ground reconstruction, set `localization_reference_topic` in
`launch_sim.launch.py` to `/uasN/localization/reference/on_air`. Vehicle then
adopts the air anchor and complete snapshots, ignores its independently received
MAVROS home and translation updates, and owns all local root edges. Bridge both
reference topics; `/tf` still need not cross the link. An empty argument is the
onboard/default mode and also supports old bag reconstruction. Do not run a fleet
root publisher alongside a Vehicle publishing its root edges.

A process restart starts a new reference generation. Consumers clear history
when a newer generation arrives. This identifier is not a controller boot ID;
actual EKF resets, changing hardware/datum, and calibration provenance still need
operational validation. Restart consumers together when replay time goes backward.
Keep COM_HOME_IN_AIR=0 for complete-home surveyed missions: it does not disable
PX4's independent GNSS/barometer altitude corrections.
