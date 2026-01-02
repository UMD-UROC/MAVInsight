from enum import Enum


class SensorTypes(Enum):
    """Enum of supported Sensor types"""

    CAMERA = "camera"
    DEFAULT = "default"
    GIMBAL = "gimbal"
    RANGEFINDER = "rangefinder"
