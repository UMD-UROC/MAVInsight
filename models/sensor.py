from models.graph_member import GraphMember
from models.sensor_types import SensorTypes

from __future__ import annotations

class Sensor(GraphMember):
    """Class that defines a sensor and its relation to its parent frame"""
    sensor_type: SensorTypes
    sensors: list[Sensor]

    def __init__(self, name:str = None,
                 frame_name:str = None, parent_frame:str = None,
                 sensor_type:SensorTypes = None,
                 *sensors:Sensor):
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.sensor_type = sensor_type
        self.sensors = list(sensors)
