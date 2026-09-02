"""Where a built scene's ground stands in a vehicle's frame.

The fleet casts its rays at one ground. `umd_uas/terrain.py` anchors that
ground on the Earth with the scene's `origin_lla`, and `umd_uas/footprint.py`
measures it against the vehicle's own localization origin, live, on every
cast. Everything drawn for the operator has to stand on that same surface, so
this module works the offset out the same way and both scene layers -- the
terrain and the buildings on it -- read it from here.

Three things decide the offset, and leaving out any one of them draws a ground
that looks right and is not:

1. The datum. A scene is anchored above mean sea level, because elevation data
   is, and a NavSatFix is above the ellipsoid. The two are a geoid separation
   apart, 33 m in Maryland.
2. The fix. The localization origin is wherever the vehicle says home is now.
   PX4 moves home when the vehicle arms, so a fix read once at startup and
   held is a different ground from the one the rays meet an hour later.
3. The survey. A fiducial capture moves the vehicle's frame against the Earth,
   and `umd_uas/footprint.py` applies that correction to every cast. The scene
   itself does not move, so a layer drawn in that frame steps the other way by
   the same amount.

The offset rides on the model's pose, so a scene that moves needs no new model
and a caller holds the bytes it built.
"""

# python imports
from typing import Optional, Sequence

# ROS2 imports
import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer, TransformListener

# ROS2 message imports
from sensor_msgs.msg import NavSatFix

# MAVInsight imports
from models.frame_utils import lla_2_enu
from models.qos_profiles import reliable_qos


# ----------------------------------------------------------------- tunables
# How far the scene has to move before it is drawn again. A live view lies on
# the ground and vanishes under a map a few centimetres over it, so this is
# centimetres rather than metres. It costs one message and no work: the model
# is held and only its pose changes, and home moves a handful of times in a
# flight rather than continuously.
REPLACE_MOVE_M = 0.02
# How far under the surveyed ground the map is drawn. What lies ON that ground
# -- the footprint outline, the live view, the marks -- then reads over the map
# instead of fighting it for the same depth. This is well under a pixel at any
# view distance an operator uses, and it moves the picture only: every
# measurement keeps the height it was computed at.
GROUND_CLEARANCE_M = 0.05
# How long a transform lookup waits. The survey is static, so it either answers
# at once or it is not there yet.
TF_TIMEOUT_S = 0.2


def as_fix(lla: Sequence[float]) -> NavSatFix:
    """A WGS84 position as a bare fix, for the ENU helpers."""
    fix = NavSatFix()
    fix.latitude, fix.longitude, fix.altitude = (float(v) for v in lla[:3])
    return fix


class FrameSurvey:
    """The standing fiducial correction on one frame, read off the tree.

    A survey says a point this vehicle reports at p is really at p + this. It
    is a statement about where the frame sits on the Earth, not about where
    anything in the frame is, so going from frame coordinates to WGS84 adds it
    and going the other way takes it off.

    The correction is the transform from the frame to its parent, which is the
    edge MAVInsight writes a survey onto. Reading the parent out of the tree
    is what `umd_uas/footprint.py` does, so no node needs that frame's name
    configured and no two can be configured differently from each other.

    No correction is the ordinary state: the edge is identity until a fiducial
    capture is localized, and zeros are what this answers then.
    """

    def __init__(self, node, frame: str) -> None:
        """Watch the survey on one frame."""
        self.node = node
        self.frame = frame
        self._parent: Optional[str] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node, spin_thread=True)

    def correction(self) -> np.ndarray:
        """The correction now, as an ENU offset. Zeros where there is none."""
        if self._parent is None:
            try:
                tree = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
                parent = (tree.get(self.frame) or {}).get("parent")
            except Exception as error:  # a malformed dump must not stop a picture
                self.node.get_logger().warn(
                    f"cannot read the frame tree: {error}",
                    throttle_duration_sec=30.0)
                return np.zeros(3)
            if not parent:
                return np.zeros(3)
            self._parent = str(parent)
            self.node.get_logger().info(
                f"fiducial correction read from {self._parent} -> {self.frame}")
        try:
            transform = self.tf_buffer.lookup_transform(
                self._parent, self.frame, RclpyTime(),
                timeout=Duration(seconds=TF_TIMEOUT_S))
        except Exception as error:  # several lookup failures share no base class
            self.node.get_logger().warn(
                f"no {self._parent} -> {self.frame} transform, so no fiducial "
                f"correction is applied: {error}", throttle_duration_sec=30.0)
            return np.zeros(3)
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])


class SceneGround:
    """The offset that puts one scene layer on the ground the rays meet.

    It holds the fix that places the reference frame, reads the survey on that
    frame, and remembers where the layer was last drawn, so a caller draws
    once for each move rather than once for each fix.
    """

    def __init__(self, node, reference_frame: str, local_fix_topic: str,
                 geoid_height_m: float) -> None:
        """Watch the fix and the survey that place `reference_frame`."""
        self.node = node
        self.reference_frame = reference_frame
        self.geoid_height = float(geoid_height_m)
        self.survey = FrameSurvey(node, reference_frame)
        self.local_fix: Optional[NavSatFix] = None
        # Where the layer stands now. A caller sets it after it publishes.
        self.drawn_at: Optional[np.ndarray] = None
        node.create_subscription(NavSatFix, local_fix_topic, self._fix_cb,
                                 reliable_qos)

    def _fix_cb(self, msg: NavSatFix) -> None:
        self.local_fix = msg

    def anchor(self, origin_lla: Sequence[float]) -> NavSatFix:
        """The scene centre in the datum a fix reports. `umd_uas/terrain.py`
        moves its tiles by the same geoid height, so the two meet."""
        latitude, longitude, mean_sea_level = (float(v) for v in origin_lla[:3])
        return as_fix((latitude, longitude, mean_sea_level + self.geoid_height))

    def offset(self, origin_lla: Sequence[float]) -> Optional[np.ndarray]:
        """Where the scene centre belongs in the reference frame, one ground
        clearance under the surveyed ground.

        The survey comes off because this goes from WGS84 into frame
        coordinates: the scene stays where it is on the Earth when the frame
        moves against it.

        None while no fix has arrived to place it. A scene at the wrong offset
        looks correct and is wrong, which is worse than an empty panel.
        """
        if self.local_fix is None:
            self.node.get_logger().warn(
                f"no fix on {self.reference_frame} yet, so the scene waits. "
                f"Placed against the wrong home it would lie metres off the "
                f"ground it belongs to.", throttle_duration_sec=10.0)
            return None
        east, north, up = lla_2_enu(self.local_fix, self.anchor(origin_lla),
                                    ignore_alt=False)
        return (np.array([east, north, up - GROUND_CLEARANCE_M])
                - self.survey.correction())

    def moved(self, offset: Optional[np.ndarray]) -> bool:
        """Whether the scene has to be drawn again to stand at this offset."""
        return offset is not None and (
            self.drawn_at is None
            or float(np.linalg.norm(offset - self.drawn_at)) >= REPLACE_MOVE_M)
