# python imports
from __future__ import annotations
from typing import Optional

# MAVInsight imports
from models.graph_member import GraphMember
from models.sensor_types import SensorTypes

class Sensor(GraphMember):
    """Class/Node that defines a sensor and its relation to its parent frame. This class
    defines how and what information will be published to Foxglove for every Sensor (i.e.
    TF Frame for position w.r.t. the Vehicle, `Marker`s representing sensor readings, etc)

    Parameters
    ----------
    offset : list[float]
        An [x, y, z] list of values that represents the static offset between the parent
        frame and this frame in meters.
    sensor_type : str
        The type of this Sensor (informed by MAVInsight.models.sensor_types.py enum).
    sensors : list[Sensor]
        A list of other Sensors that are attached to this Sensor. A common example is a
        Camera (Sensor) on a Gimbal (Sensor).
    """
    OFFSET: list[float]
    SENSOR_TYPE: SensorTypes
    SENSORS: list[str]

    # Constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info("Ingesting Sensor params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter('offset'):
            offset_param_val = self.get_parameter('offset').get_parameter_value().double_array_value
            try:
                self.OFFSET = [float(f) for f in offset_param_val]
                if len(self.OFFSET) != 3:
                    self.OFFSET = [0.0, 0.0, 0.0]
                    self.get_logger().error(f"Offset param must be exactly 3 elements long. Using, no-offset default.\n" +
                                            f"Received: {offset_param_val}")
            except ValueError as e:
                self.OFFSET = [0.0 ,0.0 ,0.0]
                self.get_logger().error(f"Unable to interpret offset param elements as floats. Using no-offset default.\n" +
                                        f"Received: {offset_param_val}\n" +
                                        f"Error: {e}")
        else:
            self.default_parameter_warning("offset")
            self.OFFSET = [0.0, 0.0, 0.0]

        if self.has_parameter('sensor_type'):
            self.SENSOR_TYPE = SensorTypes(self.get_parameter("sensor_type").get_parameter_value().string_value)
        else:
            self.default_parameter_warning("sensor_type")
            self.SENSOR_TYPE = SensorTypes.DEFAULT

        if self.has_parameter("sensors"):
            self.SENSORS = list(self.get_parameter("sensors").get_parameter_value().string_array_value)
        else:
            self.SENSORS = []

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t1 = self._tab_char * tab_depth
        t2 = t1 + self._tab_char
        sensors_string = "[]" if len(self.SENSORS) == 0 else "\n"
        return (
            f"{t1}{self.DISPLAY_NAME} | Sensor {self.SENSOR_TYPE.name}\n" +
            f"{t2}Transform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n" +
            f"{t2}Static offset from parent: (x: {self.OFFSET[0]}, y: {self.OFFSET[1]}, z: {self.OFFSET[2]})\n" +
            extra_fields +
            f"{t2}Sensors: {sensors_string}" +
            ("\n".join(t2 + self._tab_char + s for s in self.SENSORS))
        )

    def __str__(self):
        return self._format()

class Camera(Sensor):
    """Class/Node defining a Camera and how/what visualization info will be published for
    Foxglove.

    Parameters
    ----------
    cam_info_topic : str
        The str name of the topic carrying the camera info.
    """
    CAM_INFO_TOPIC: Optional[str]

    # Constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info("Ingesting Camera params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter('cam_info_topic'):
            self.CAM_INFO_TOPIC = self.get_parameter('cam_info_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning("cam_info_topic")
            self.CAM_INFO_TOPIC = "camera_info"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        camera_fields = f"{t}Camera info topic: {self.CAM_INFO_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=camera_fields)

    def __str__(self):
        return self._format()

class Gimbal(Sensor):
    """Class/Node defining a Gimbal and how/what visualization info will be published for
    Foxglove.

    Parameters
    ----------
    orientation_topic : str
        The str name of the topic carrying the gimbal orientation data.
    """
    ORIENTATION_TOPIC:str

    # Constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info("Ingesting Camera params...")

        # Ingest ROS parameters. Notify user when defaults are being used
        if self.has_parameter("orientation_topic"):
            self.ORIENTATION_TOPIC = self.get_parameter("orientation_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("orientation_topic")
            self.ORIENTATION_TOPIC = "gimbal_orientation"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        gimbal_fields = f"{t}Orientation topic: {self.ORIENTATION_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=gimbal_fields)

    def __str__(self):
        return self._format()

class Rangefinder(Sensor):
    """Class/Node defining a Rangefinder and how/what visualization info will be published
    for Foxglove.

    Parameters
    ----------
    range_topic : str
        The name of the topic carrying the range info.
    """
    RANGE_TOPIC: Optional[str]

    # Constructors
    def __init__(self):
        super().__init__()
        self.get_logger().info("Ingesting Rangefinder params...")

        # Ingest ROS parameters. Notify user when defaults are being used.
        if self.has_parameter('range_topic'):
            self.RANGE_TOPIC = self.get_parameter('range_topic').get_parameter_value().string_value
        else:
            self.default_parameter_warning("range_topic")
            self.RANGE_TOPIC = "rangefinder"

    def _format(self, tab_depth:int=0, extra_fields:str="") -> str:
        t = self._tab_char * (tab_depth + 1)
        rangefinder_fields = f"{t}Range topic: {self.RANGE_TOPIC}\n" + extra_fields
        return super()._format(tab_depth=tab_depth, extra_fields=rangefinder_fields)

    def __str__(self):
        return self._format()
