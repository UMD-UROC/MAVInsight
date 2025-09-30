from enum import Enum

class SensorTypes(Enum):
    """Enum of supported Sensor types"""
    CAMERA = "camera"
    GIMBAL = "gimbal"
    RANGEFINDER = "rangefinder"
