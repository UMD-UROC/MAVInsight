# python imports
import yaml
from __future__ import annotations
from pathlib import Path

# ROS2 imports
from ament_index_python import get_package_share_directory

# MAVInsight imports
from models.graph_member import GraphMember
from models.sensor_types import SensorTypes

class Sensor(GraphMember):
    """Class that defines a sensor and its relation to its parent frame

    Attributes:
        offset      An (x, y, z) tuple that represents the static offset between the parent frame and this frame.
        sensor_type The type of this Sensor (informed by MAVInsight.models.sensor_types.py enum).
        sensors     A list of other Sensors that are attached to this Sensor. Common example is Camera (Sensor) on a Gimbal (Sensor).
    """
    offset: tuple[float, float, float]
    sensor_type: SensorTypes
    sensors: list[Sensor]

    # Constructors
    def __init__(self,
                 frame_name:str = None,
                 name:str = None,
                 offset:list[float] = [0.0, 0.0, 0.0],
                 parent_frame:str = None,
                 sensor_type:SensorTypes = None,
                 sensors:list[str] = []):
        if len(offset) != 3:
            raise ValueError(f"Sensor offset must be exactly 3 elements. Received: {offset}")
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.offset = offset
        self.sensor_type = sensor_type
        self.sensors = []
        for sensor_path in sensors:
            self.sensors.append(sensor_factory(sensor_path))

class Camera(Sensor):
    """Class defining a Camera for Foxglove Viz. Extends Sensor.

    Attributes:
        cam_info_topic  The name of the topic carrying the camera info.
    """
    cam_info_topic: str

    # Constructors
    def __init__(self,
                 cam_info_topic:str = None,
                 frame_name:str = None,
                 name:str = None,
                 offset:list[float] = [0.0, 0.0, 0.0],
                 parent_frame:str = None,
                 sensor_type:SensorTypes = None):
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type)
        self.cam_info_topic = cam_info_topic

    def __init__(self, config_params:dict):
        param_reqs = ["cam_info_topic", "frame_name", "name", "offset", "parent_frame", "sensor_type"]
        if not set(param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params to build Camera. Must have all of {param_reqs}")

        super().__init__(frame_name=config_params["frame_name"],
                         name=config_params["name"],
                         offset=config_params["offset"],
                         parent_frame=config_params["parent_frame"],
                         sensor_type=SensorTypes(config_params["sensor_type"]))
        self.cam_info_topic = config_params["camera_info_topic"]

class Gimbal(Sensor):
    """Class defining a Gimbal for Foxglove Viz. Extends Sensor.

    Attributes:
        orientation_topic  The name of the topic carrying the gimbal orientation data.
    """
    orientation_topic: str

    # Constructors
    def __init__(self,
                 frame_name:str = None,
                 name:str = None,
                 offset:list[float] = [0.0, 0.0, 0.0],
                 orientation_topic:str = None,
                 parent_frame:str = None,
                 sensor_type:SensorTypes = None,
                 sensors:list[str] = []):
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type, sensors=sensors)
        self.orientation_topic = orientation_topic

    def __init__(self, config_params:dict):
        param_reqs = ["frame_name", "name", "offset", "orientation_topic", "parent_frame", "sensor_type", "sensors"]
        if not set(param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params to build Gimbal. Must have all of {param_reqs}")

        super().__init__(frame_name=config_params["frame_name"],
                         name=config_params["name"],
                         offset=config_params["offset"],
                         parent_frame=config_params["parent_frame"],
                         sensor_type=SensorTypes(config_params["sensor_type"]),
                         sensors=config_params["sensors"])
        self.orientation_topic = config_params["orientation_topic"]

class Rangefinder(Sensor):
    """Class defining a Rangefinder for Foxglove Viz. Extends Sensor.

    Attributes:
        range_topic  The name of the topic carrying the range info.
    """
    range_topic: str

    # Constructors
    def __init__(self,
                 frame_name:str = None,
                 name:str = None,
                 offset:tuple[float, float, float] = (0,0,0),
                 parent_frame:str = None,
                 range_topic:str = None,
                 sensor_type:SensorTypes = None):
        super().__init__(frame_name=frame_name, name=name, offset=offset, parent_frame=parent_frame, sensor_type=sensor_type)
        self.range_topic = range_topic

    def __init__(self, config_params:dict):
        param_reqs = ["frame_name", "name", "offset", "parent_frame", "range_topic", "sensor_type"]
        if not set(param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params to build Rangefinder. Must have all of {param_reqs}")

        super().__init__(frame_name=config_params["frame_name"],
                         name=config_params["name"],
                         offset=config_params["offset"],
                         parent_frame=config_params["parent_frame"],
                         sensor_type=SensorTypes(config_params["sensor_type"]))
        self.range_topic = config_params["range_topic"]

def sensor_factory(filename:str) -> Sensor:
    """Factory for producing a (supported) sensor defined in {TODO: make sensor config folder}"""
    path = Path(get_package_share_directory("mavinsight")) / "sensors" / filename

    if not path.is_file():
        raise ValueError(f"No configs found under {path}")

    with open(path, 'r', encoding='utf-8') as sensor_config:
        match sensor_config["sensor_type"]:
            case SensorTypes.CAMERA.value:
                return (Camera(sensor_config))
            case SensorTypes.GIMBAL.value:
                return (Gimbal(sensor_config))
            case SensorTypes.RANGEFINDER.value:
                return (Rangefinder(sensor_config))
            case _:
                raise ValueError(f"Unrecognized Sensor type: {sensor_config["sensor_type"]}")
