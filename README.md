# MAVInsight
The `mavinsight` package is a ROS2 Package configured to build a model of a Vehicle for later visualization in Foxglove.

## Package Structure
- `MAVInsight/` - This repo.
  - `mavinsight/` - Source directory for the Nodes of this repo.
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
