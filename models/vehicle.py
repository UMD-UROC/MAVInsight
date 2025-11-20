# python imports
from __future__ import annotations

# ROS imports
import rclpy

# MAVInsight Imports
from models.graph_member import GraphMember
from models.platforms import Platforms

class Vehicle(GraphMember):
    """Class/Node that defines a generic vehicle (typically a drone) and its sensors.
    This Class defines what information should be published for all Vehicles. (i.e.
    TF Frame for position, velocity `Marker`, etc).

    Parameters
    ----------
    location_topic : str
        The ROS topic that this `Vehicle` should look at to get its location data.
    platform : Platforms | str
        The type of this `Vehicle` (informed by `MAVInsight.models.platforms.py` enum).
    sensors : list[Sensor]
        A list of `Sensors` attached to this vehicle.
    """
    LOCATION_TOPIC: str
    PLATFORM: Platforms
    SENSORS: list[str]

    # Constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info(f"Ingesting Vehicle params...")

        # Ingest ROS parameters. Notify user when defaults are being used.
        if self.has_parameter('location_topic'):
            self.LOCATION_TOPIC = self.get_parameter('location_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning('location_topic')
            self.LOCATION_TOPIC = "gps"

        if self.has_parameter("platform"):
            self.PLATFORM = Platforms(self.get_parameter("platform").get_parameter_value().string_value)
        else:
            self.default_parameter_warning('platform')
            self.PLATFORM = Platforms.DEFAULT

        if self.has_parameter("sensors"):
            self.SENSORS = list(self.get_parameter("sensors").get_parameter_value().string_array_value)
        else:
            self.SENSORS = []

        self.get_logger().info(self._format())

    def _format(self, tab_depth:int=0) -> str:
        t1 = self._tab_char * tab_depth
        t2 = t1 + self._tab_char
        sensors_string = "[]" if len(self.SENSORS) == 0 else "\n"
        return ( f"Vehicle Structure ({self.get_name()}):\n" +
            f"{t1}{self.DISPLAY_NAME} | Vehicle ({self.PLATFORM.name})\n" +
            f"{t2}Transform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n" +
            f"{t2}Location Topic: {self.LOCATION_TOPIC}\n" +
            f"{t2}Sensors: {sensors_string}" +
            ("\n".join(t2 + self._tab_char + s for s in self.SENSORS))
        )

    def __str__(self):
        return self._format()
