from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor

from __future__ import annotations

class Vehicle(GraphMember):
    """Class that defines a drone platform and its sensors"""
    platform: Platforms
    sensors: list[Sensor]

    def __init__(self, name:str = None, frame_tf:str = None, platform:Platforms = None, *sensors:Sensor):
        super().__init__(name, frame_tf)
        self.platform = platform
        self.sensors = list(sensors)