from enum import Enum


class Platforms(Enum):
    """Enum of supported platforms"""
    DEFAULT = "default"
    FIXED_WING = "fixed_wing"
    HEX_COPTER = "hex"
    QUAD_COPTER = "quad"
    STATIONARY = "stationary"
    VTOL = "vtol"
