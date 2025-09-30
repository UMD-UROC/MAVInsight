from models.graph_member import GraphMember
from models.sensor_types import SensorTypes

from __future__ import annotations

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
    def __init__(self, name:str = None,
                 frame_name:str = None,
                 parent_frame:str = None,
                 offset:tuple[float, float, float] = (0,0,0),
                 sensor_type:SensorTypes = None,
                 *sensors:Sensor):
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.offset = offset
        self.sensor_type = sensor_type
        self.sensors = list(sensors)
