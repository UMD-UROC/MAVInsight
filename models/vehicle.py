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
        print(f"Successfully built Vehicle: {self.name}")

    def __init__(self, config_params: dict):
        field_reqs = ["name", "frame_name", "location_topic", "parent_frame", "platform"]
        if not set(field_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Unable to create Vehicle from dict: must contain all of the following keys: {field_reqs}")

        super().__init__(name=config_params["name"],
                         frame_name=config_params["frame_name"],
                         parent_frame=config_params["parent_frame"])
        self.location_topic = config_params["location_topic"]
        self.platform = Platforms(config_params["platform"])
        self.sensors = []
        if "sensors" in config_params:
            for sensor_path in config_params["sensors"]:
                try:
                    self.sensors.append(sensor_factory(sensor_path))
                except Exception as e:
                    print(f"\033[31mFAILED building Sensor from file {sensor_path}: {e}\033[0m")
        print(f"Successfully built Vehicle: {self.name}")

    def _format(self, tab_depth:int=0) -> str:
        t1 = self.tab_char * tab_depth
        t2 = t1 + self.tab_char
        sensors_string = "[]" if len(self.sensors) == 0 else "\n"
        return (
            f"{t1}{self.name} | Vehicle ({self.platform.name})\n" +
            f"{t2}Transform: {self.parent_frame} -> {self.frame_name}\n" +
            f"{t2}Location Topic: {self.location_topic}\n" +
            f"{t2}Sensors: {sensors_string}" +
            ("\n".join(s._format(tab_depth=tab_depth + 2) for s in self.sensors))
        )

    def __str__(self):
        return self._format()
