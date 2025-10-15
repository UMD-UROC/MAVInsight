from __future__ import annotations
from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor
from models.sensor import sensor_factory

class Vehicle(GraphMember):
    """Class that defines a drone platform and its Sensors

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Vehicle` would need
        to be considered "valid". Inherited from `GraphMember` and extended.

    Attributes
    ----------
    location_topic : str
        The topic that this vehicle should look at to get its location data.
    platform : Platforms | str
        The type of this Vehicle (informed by MAVInsight.models.platforms.py enum).
    sensors : list[Sensor]
        A list of Sensors attached to this vehicle.
    """
    location_topic: str
    platform: Platforms
    sensors: list[Sensor]

    param_reqs:list[str] = GraphMember.param_reqs + ["location_topic", "parent_frame", "platform"]

    # Constructors
    def __init__(self, name:str=None, frame_name:str=None, location_topic:str=None, parent_frame:str="map", platform:Platforms=None, sensors:list[Sensor]=[]):
        """Basic Vehicle constructor. No error/input checking/scrubbing."""
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.location_topic = location_topic
        self.platform = platform
        self.sensors = sensors
        self.param_reqs = self.param_reqs + self.param_reqs
        print(f"Successfully built Vehicle: {self.name}")

    @classmethod
    def from_dict(clazz, config_params: dict) -> Vehicle:
        # fill in a default parent frame, if none other is specified for a vehicle
        if ("parent_frame" not in config_params.keys()):
            config_params["parent_frame"] = "map"

        # check for required Vehicle Params
        if not clazz._dict_meets_reqs(config_params):
            raise ValueError(f"Not enough params in dict to create Vehicle. Must have all of: {clazz.param_reqs}")

        return clazz(name=config_params["name"],
                        frame_name=config_params["frame_name"],
                        location_topic=config_params["location_topic"],
                        parent_frame=config_params["parent_frame"],
                        platform=Platforms(config_params["platform"]),
                        sensors=Sensor._make_from_file_list(config_params.get("sensors", [])))

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
