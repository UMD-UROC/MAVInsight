# python imports
from __future__ import annotations
from pathlib import Path
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
        A list of required parameters/keys that a dict-encoded version of a `Sensor` would need
        to be considered "valid".

    Attributes
    ----------
    offset : list[float]
        An [x, y, z] list of values that represents the static offset between the parent frame and this frame in meters.
    sensor_type : str | SensorTypes
        The type of this Sensor (informed by MAVInsight.models.sensor_types.py enum).
    sensors : list[Sensor]
        A list of other Sensors that are attached to this Sensor. Common example is Camera (Sensor) on a Gimbal (Sensor).
    """
    offset: list[float, float, float]
    sensor_type: SensorTypes
    sensors: list[Sensor]

    param_reqs = ["offset", "sensor_type"]

    # Constructors
    def __init__(self, frame_name:str=None, name:str=None, offset:list[float]=[0.0,0.0,0.0], parent_frame:str=None, sensor_type:SensorTypes=None, sensors:list[Sensor]=[]):
        """Basic Sensor constructor, no error/input checking/scrubbing"""
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.offset = offset
        self.sensor_type = sensor_type
        self.sensors = sensors

    def check_dict(self, config_params):
        if ("offset" not in config_params.keys()) or (len(config_params["offset"]) == 0):
            config_params["offset"] = [0.0,0.0,0.0]

        if not set(Sensor.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create Sensor. Must have all of: {Sensor.param_reqs}")

        # parameter checking
        if len(config_params["offset"]) != 3:
            raise ValueError(f"Sensor offset must be exactly 3 elements. Received: {config_params['offset']}")
        if any([type(val) != float for val in config_params["offset"]]):
            raise ValueError(f"Sensor offset values must be floats. Received: {config_params['offset']}")

        self.offset = config_params["offset"]
        self.sensor_type = SensorTypes(config_params["sensor_type"])
        self.sensors = Sensor._make_from_file_list(config_params.get("sensors", []))

        super().check_dict(config_params)

    @staticmethod
    def _make_from_file_list(file_list:list[str]) -> list[Sensor]:
        s = []
        for sensor_path in file_list:
            #try:
            s.append(sensor_factory(sensor_path))
            # except Exception as e:
            #     print(f"\033[31mFAILED building Sensor from file {sensor_path}: {e}\033[0m")
        return s

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t1 = self.tab_char * tab_depth
        t2 = t1 + self.tab_char
        sensors_string = "[]" if len(self.sensors) == 0 else "\n"
        return (
            f"{t1}{self.name} | Sensor {self.sensor_type.name}\n" +
            f"{t2}Transform: {self.parent_frame} -> {self.frame_name}\n" +
            f"{t2}Static offset from parent: (x: {self.offset[0]}, y: {self.offset[1]}, z: {self.offset[2]})\n" +
            extra_fields +
            f"{t2}Sensors: {sensors_string}" +
            "\n".join(s._format(tab_depth=tab_depth + 2) for s in self.sensors)
        )

    def __str__(self):
        return self._format()

class Camera(Sensor):
    """Class defining a Camera for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Camera` would need
        to be considered "valid".

    Attributes
    ----------
    cam_info_topic : str
        The str name of the topic carrying the camera info.
    """
    cam_info_topic: str

    param_reqs: list[str] = ["cam_info_topic"]

    # Constructors
    def __init__(self, cam_info_topic:str=None, frame_name:str=None, name:str=None, offset:list[float]=[0.0, 0.0, 0.0], parent_frame:str=None, sensor_type:SensorTypes=None):
        """Basic Camera constructor, no error/input checking/scrubbing"""
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type)
        self.cam_info_topic = cam_info_topic

    def check_dict(self, config_params):
        if not set(Camera.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create Camera. Must have all of: {Camera.param_reqs}")

        self.cam_info_topic = config_params["cam_info_topic"]

        super().check_dict(config_params)

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self.tab_char * (tab_depth + 1)
        camera_fields = f"{t}Camera info topic: {self.cam_info_topic}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=camera_fields)

    def __str__(self):
        return self._format()

class Gimbal(Sensor):
    """Class defining a Gimbal for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Gimbal` would need
        to be considered "valid".

    Attributes
    ----------
    orientation_topic : str
        The str name of the topic carrying the gimbal orientation data.
    """
    orientation_topic: str

    param_reqs: list[str] = ["orientation_topic"]

    # Constructors
    def __init__(self, frame_name:str = None, name:str = None, offset:list[float] = [0.0, 0.0, 0.0], orientation_topic:str = None, parent_frame:str = None, sensor_type:SensorTypes = None, sensors:list[str] = []):
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type, sensors=sensors)
        self.orientation_topic = orientation_topic

    def check_dict(self, config_params):
        if not set(Gimbal.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create Gimbal. Must have all of: {Gimbal.param_reqs}")

        self.orientation_topic = config_params["orientation_topic"]
        self.sensors = Sensor._make_from_file_list(config_params.get("sensors", []))

        super().check_dict(config_params)

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self.tab_char * (tab_depth + 1)
        gimbal_fields = f"{t}Orientation topic: {self.orientation_topic}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=gimbal_fields)

    def __str__(self):
        return self._format()

class Rangefinder(Sensor):
    """Class defining a Rangefinder for Foxglove Viz. Extends Sensor.

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Rangefinder` would need
        to be considered "valid".

    Attributes
    ----------
    range_topic : str
        The name of the topic carrying the range info.
    """
    range_topic: str

    param_reqs: list[str] = ["range_topic"]

    # Constructors
    def __init__(self, frame_name:str = None, name:str = None, offset:list[float, float, float]=[0.0,0.0,0.0], parent_frame:str = None, range_topic:str = None, sensor_type:SensorTypes = None):
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type)
        self.range_topic = range_topic

    def check_dict(self, config_params):
        if not set(Rangefinder.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create Rangefinder. Must have all of: {Rangefinder.param_reqs}")

        self.range_topic = config_params["range_topic"]

        super().check_dict(config_params)

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self.tab_char * (tab_depth + 1)
        rangefinder_fields = f"{t}Range topic: {self.range_topic}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=rangefinder_fields)

    def __str__(self):
        return self._format()

def sensor_factory(filename:str) -> Sensor:
    """Factory for producing a (supported) sensor defined in mavinsight/sensors/*.yaml."""

    if filename.startswith("_test/"):
        filename = filename.removeprefix("_test/")
        filename = Path(__file__).resolve().parent.parent / "test/sensors" / filename

    path = Path(filename)
    if not path.is_absolute():
        path = Path(get_package_share_directory("mavinsight")) / "sensors" / path

    if not path.is_file():
        raise FileNotFoundError(f"No configs found under {path}")

    with open(path, 'r', encoding='utf-8') as sensor_file:
        sensor_config = yaml.safe_load(sensor_file)
        if type(sensor_config) is not dict:
            raise ValueError(f"Error parsing {path} as yaml in sensor_factory. Sensor configs must be yaml-encoded.")

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
