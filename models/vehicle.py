from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor

class Vehicle(GraphMember):
    """Class that defines a drone platform and its sensors

    Attributes:
        platform        The type of this Vehicle (informed by MAVInsight.models.platforms.py enum).
        sensors         A list of Sensors attached to this vehicle.
    """
    platform: Platforms
    sensors: list[Sensor]

    # Constructors
    def __init__(self, name:str = None,
                 frame_name:str = None, parent_frame:str = "map",
                 platform:Platforms = None,
                 *sensors:Sensor):
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.platform = platform
        self.sensors = list(sensors)
