from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor

from __future__ import annotations

class Vehicle(GraphMember):
    """Class that defines a drone platform and its sensors"""
    platform: Platforms
    sensors: list[Sensor]

    def __init__(self, name:str = None,
                 frame_name:str = None, parent_frame:str = "map",
                 platform:Platforms = None,
                 *sensors:Sensor):
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.platform = platform
        self.sensors = list(sensors)
