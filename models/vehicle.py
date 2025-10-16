# python imports
from __future__ import annotations
from pathlib import Path
import yaml

# ROS2 imports
from ament_index_python import get_package_share_directory

# MAVInsight Imports
from models.graph_member import GraphMember
from models.platforms import Platforms
from models.sensor import Sensor

class Vehicle(GraphMember):
    """Class that defines a drone platform and its sensors

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `Vehicle` would need,
        in addition to the `GraphMember's`, to be considered "valid".

    Attributes
    ----------
    location_topic : str
        The ROS topic that this `Vehicle` should look at to get its location data.
    platform : Platforms | str
        The type of this `Vehicle` (informed by `MAVInsight.models.platforms.py` enum).
    sensors : list[Sensor]
        A list of `Sensors` attached to this vehicle.
    """
    location_topic: str
    platform: Platforms
    sensors: list[Sensor]

    param_reqs:list[str] = ["location_topic", "platform"]

    # Constructors
    def __init__(self, name:str=None, frame_name:str=None, location_topic:str=None, parent_frame:str="map", platform:Platforms=None, sensors:list[Sensor]=[]):
        """Basic Vehicle constructor. No error/input checking/scrubbing."""
        super().__init__(name=name, frame_name=frame_name, parent_frame=parent_frame)
        self.location_topic = location_topic
        self.platform = platform
        self.sensors = sensors

    def check_dict(self, config_params: dict):
        """
        A faux-constructor. Used to offload the parameter checking of a dict-encoded
        `GraphMember` object to each level of the class heirarchy of `GraphMember`
        and its subclasses.
        """

        # "guard" the case of a missing parent frame in the dict, default to "map"
        if ("parent_frame" not in config_params.keys()):
            config_params["parent_frame"] = "map"

        # check for required Vehicle params
        if not set(Vehicle.param_reqs).issubset(set(config_params.keys())):
            raise ValueError(f"Not enough params in dict to create Vehicle. Must have all of: {Vehicle.param_reqs}")

        # set the Vehicle params
        self.location_topic = config_params["location_topic"]
        self.platform = Platforms(config_params["platform"])
        self.sensors = Sensor._make_from_file_list(config_params.get("sensors", []))

        # let the super check its own params
        super().check_dict(config_params)

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

def vehicle_factory(filename:str) -> Vehicle:
    """Factory for producing a (supported) vehicle defined in mavinsight/vehicles/*.yaml."""

    # this is necessary for unit tests and encoding test models in the test directory,
    # regardless of abs path to this package
    if filename.startswith("_test/"):
        filename = filename.removeprefix("_test/")
        filename = Path(__file__).resolve().parent.parent / "test/vehicles" / filename

    # assume the node is looking for the name of a vehicle config found in the shared config
    # directory of this ROS package
    path = Path(filename)
    if not path.is_absolute():
        path = Path(get_package_share_directory("mavinsight")) / "vehicles" / path

    if not path.is_file():
        raise FileNotFoundError(f"No configs found under {path}")

    with open(path, 'r', encoding='utf-8') as sensor_file:
        vehicle_config = yaml.safe_load(sensor_file)
        if type(vehicle_config) is not dict:
            raise TypeError(f"Error parsing {path} as yaml in vehicle_factory. Vehicle configs must be yaml-encoded.")

        return Vehicle.from_dict(vehicle_config)
