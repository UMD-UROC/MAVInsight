# python imports
from __future__ import annotations
from pathlib import Path
from typing import Optional
import yaml

# ROS2 imports
from ament_index_python import get_package_share_directory

# MAVInsight imports
from models.graph_member import GraphMember
from models.sensor_types import SensorTypes

class Sensor(GraphMember):
    """Class that defines a sensor and its relation to its parent frame

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Sensor` would need,
        in addition to the `GraphMember's`, to be considered "valid".

    Attributes
    ----------
    offset : list[float]
        An [x, y, z] list of values that represents the static offset between the parent frame and this frame in meters.
    sensor_type : str | SensorTypes
        The type of this Sensor (informed by MAVInsight.models.sensor_types.py enum).
    sensors : list[Sensor]
        A list of other Sensors that are attached to this Sensor. Common example is Camera (Sensor) on a Gimbal (Sensor).
    """
    OFFSET: list[float]
    SENSOR_TYPE: SensorTypes
    SENSORS: list[str]

    # Constructors
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Ingesting Sensor params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter('offset'):
            offset_param_val = self.get_parameter('offset').get_parameter_value().double_array_value
            try:
                self.OFFSET = [float(f) for f in offset_param_val]
                if len(self.OFFSET) != 3:
                    self.OFFSET = [0.0, 0.0, 0.0]
                    self.get_logger().error(f"Offset param must be exactly 3 elements long. Using, no-offset default.\n" +
                                            f"Received: {offset_param_val}")
            except ValueError as e:
                self.OFFSET = [0.0 ,0.0 ,0.0]
                self.get_logger().error(f"Unable to interpret offset param elements as floats. Using no-offset default.\n" +
                                        f"Received: {offset_param_val}\n" +
                                        f"Error: {e}")
        else:
            self.default_parameter_warning("offset")
            self.OFFSET = [0.0, 0.0, 0.0]

        if self.has_parameter('sensor_type'):
            self.SENSOR_TYPE = SensorTypes(self.get_parameter("sensor_type").get_parameter_value().string_value)
        else:
            self.default_parameter_warning("sensor_type")
            self.SENSOR_TYPE = SensorTypes.DEFAULT

        if self.has_parameter("sensors"):
            self.SENSORS = list(self.get_parameter("sensors").get_parameter_value().string_array_value)
        else:
            self.SENSORS = []

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t1 = self._tab_char * tab_depth
        t2 = t1 + self._tab_char
        sensors_string = "[]" if len(self.SENSORS) == 0 else "\n"
        return (
            f"{t1}{self.DISPLAY_NAME} | Sensor {self.SENSOR_TYPE.name}\n" +
            f"{t2}Transform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n" +
            f"{t2}Static offset from parent: (x: {self.OFFSET[0]}, y: {self.OFFSET[1]}, z: {self.OFFSET[2]})\n" +
            extra_fields +
            f"{t2}Sensors: {sensors_string}" +
            ("\n".join(t2 + self._tab_char + s for s in self.SENSORS))
        )

    def __str__(self):
        return self._format()

class Camera(Sensor):
    """Class defining a Camera for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Camera` would need,
        in addition to the `Sensor's`, to be considered "valid".

    Attributes
    ----------
    cam_info_topic : str
        The str name of the topic carrying the camera info.
    """
    CAM_INFO_TOPIC: Optional[str]

    # Constructors
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Ingesting Camera params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter('cam_info_topic'):
            self.CAM_INFO_TOPIC = self.get_parameter('cam_info_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning("cam_info_topic")
            self.CAM_INFO_TOPIC = "camera_info"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        camera_fields = f"{t}Camera info topic: {self.CAM_INFO_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=camera_fields)

    def __str__(self):
        return self._format()

class Gimbal(Sensor):
    """Class defining a Gimbal for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Gimbal` would need,
        in addition to the `Sensor's`, to be considered "valid".

    Attributes
    ----------
    orientation_topic : str
        The str name of the topic carrying the gimbal orientation data.
    """
    ORIENTATION_TOPIC:str

    # Constructors
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Ingesting Camera params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter("orientation_topic"):
            self.ORIENTATION_TOPIC = self.get_parameter("orientation_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("orientation_topic")
            self.ORIENTATION_TOPIC = "gimbal_orientation"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        gimbal_fields = f"{t}Orientation topic: {self.ORIENTATION_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=gimbal_fields)

    def __str__(self):
        return self._format()

class Rangefinder(Sensor):
    """Class defining a Rangefinder for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Rangefinder` would need,
        in addition to the `Sensor's`, to be considered "valid".

    Attributes
    ----------
    range_topic : str
        The name of the topic carrying the range info.
    """
    RANGE_TOPIC: Optional[str]

    # Constructors
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("Ingesting Rangefinder params...")

        # Ingest ROS parameters. Notify user when defaults are being used.
        if self.has_parameter('range_topic'):
            self.RANGE_TOPIC = self.get_parameter('range_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning("range_topic")
            self.RANGE_TOPIC = "rangefinder"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        rangefinder_fields = f"{t}Range topic: {self.RANGE_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=rangefinder_fields)

    def __str__(self):
        return self._format()

def sensor_factory(filename:str) -> Sensor:
    """Factory for producing a (supported) sensor defined in mavinsight/sensors/*.yaml."""

    # this is necessary for unit tests and encoding test models in the test directory,
    # regardless of abs path to this package
    if filename.startswith("_test/"):
        filename = filename.removeprefix("_test/")
        filename = (Path(__file__).resolve().parent.parent / "test" / filename).as_posix()

    # assume the node is looking for the name of a sensor config found in the shared config
    # directory of this ROS package
    path = Path(filename)
    if not path.is_absolute():
        path = Path(get_package_share_directory("mavinsight")) / "sensors" / path

    if not path.is_file():
        raise FileNotFoundError(f"No configs found under {path}")

    # call the appropriate constructor based on the type of sensor provided
    with open(path, 'r', encoding='utf-8') as sensor_file:
        sensor_config = yaml.safe_load(sensor_file)
        if type(sensor_config) is not dict:
            raise TypeError(f"Error parsing {path} as yaml in sensor_factory. Sensor configs must be yaml-encoded.")

        if 'sensor_type' not in sensor_config:
            raise ValueError(f"Sensor config file {path} does not contain sensor type parameter.")

        match sensor_config["sensor_type"]:
            case SensorTypes.CAMERA.value:
                return (Camera.from_dict(sensor_config))
            case SensorTypes.GIMBAL.value:
                return (Gimbal.from_dict(sensor_config))
            case SensorTypes.RANGEFINDER.value:
                return (Rangefinder.from_dict(sensor_config))
            case _:
                raise ValueError(f"Unrecognized Sensor type: {sensor_config['sensor_type']}")
