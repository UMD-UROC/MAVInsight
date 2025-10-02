from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor
from models.sensor import sensor_factory

class Vehicle(GraphMember):
    """Class that defines a drone platform and its sensors

    Attributes:
        location_topic  The topic that this vehicle should look at to get its location data.
        platform        The type of this Vehicle (informed by MAVInsight.models.platforms.py enum).
        sensors         A list of Sensors attached to this vehicle.
    """
    location_topic:str
    platform: Platforms
    sensors: list[Sensor]

    # Constructors
    def __init__(self, name:str = None,
                 frame_name:str = None,
                 location_topic:str = None,
                 parent_frame:str = "map",
                 platform:Platforms = None,
                 sensors:list[Sensor] = []):
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.location_topic = location_topic
        self.platform = platform
        self.sensors = sensors

    def __init__(self, config_params: dict):
        field_reqs = ["name", "frame_name", "location_topic", "parent_frame", "platform", "sensors"]
        if not set(field_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Unable to create Vehicle from dict: must contain all of the following keys: {field_reqs}")

        super().__init__(name=config_params["name"],
                         frame_name=config_params["frame_name"],
                         parent_frame=config_params["parent_frame"])
        self.location_topic = config_params["location_topic"]
        self.platform = Platforms(config_params["platform"])
        self.sensors = []
        for sensor_path in config_params["sensors"]:
            self.sensors.append(Sensor.sensor_factory(sensor_path))
