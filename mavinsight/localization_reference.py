"""Atomic PX4 home composition and stepwise localization state.

MAVROS inputs are ENU and ellipsoid heights. Application HOME is a fixed
geographic anchor chosen once, not PX4's mutable RTL/navigation home. All
geographic placement uses the complete pair H-h from one HomePosition.
"""
from bisect import bisect_right
from dataclasses import dataclass
import json
import math
import uuid

import pymap3d as pm


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class StateHistory:
    """Bounded sample-and-hold history; late packets never roll back latest.

    A zero query requests latest. Before the first activation there is no
    state: callers must retry/drop rather than silently use a future value.
    """
    def __init__(self, limit=4096):
        self.limit = limit
        self._times = []
        self._values = []

    def add(self, time_ns, value):
        index = bisect_right(self._times, time_ns)
        if index and self._times[index - 1] == time_ns:
            self._values[index - 1] = value
        else:
            self._times.insert(index, time_ns)
            self._values.insert(index, value)
        excess = len(self._times) - self.limit
        if excess > 0:
            del self._times[:excess]
            del self._values[:excess]

    def contains(self, time_ns):
        index = bisect_right(self._times, time_ns)
        return bool(index and self._times[index - 1] == time_ns)

    def at(self, time_ns=0):
        index = len(self._times) if not time_ns else bisect_right(self._times, time_ns)
        return self._values[index - 1] if index else None


@dataclass(frozen=True)
class ReferenceState:
    generation: str
    stamp: int
    anchor: tuple
    ekf_offset: tuple
    correction: tuple
    raw_frame: str
    home_frame: str
    ekf_frame: str

    def encode(self):
        return json.dumps(dict(schema=1, **self.__dict__), allow_nan=False)

    @classmethod
    def decode(cls, data):
        values = json.loads(data)
        if values.pop('schema') != 1:
            raise ValueError('unsupported localization reference schema')
        for key in ('anchor', 'ekf_offset', 'correction'):
            values[key] = tuple(float(v) for v in values[key])
            if len(values[key]) != 3 or not all(math.isfinite(v) for v in values[key]):
                raise ValueError('invalid localization reference vector')
        if not isinstance(values['stamp'], int) or values['stamp'] < 0:
            raise ValueError('invalid localization reference timestamp')
        return cls(**values)

    def frame_anchor(self, frame, corrected=True):
        if frame == self.ekf_frame:
            offset = self.ekf_offset
        elif frame in (self.home_frame, self.raw_frame):
            offset = (0., 0., 0.)
        else:
            raise ValueError('frame is not in this localization reference')
        if corrected and frame != self.raw_frame:
            offset = tuple(a + b for a, b in zip(offset, self.correction))
        return tuple(pm.enu2geodetic(*offset, *self.anchor, deg=True))


class LocalizationReference:
    """Compose mutable home pairs into a stable application reference.

    No threshold suppresses genuine reference movement. Millimetre MAVLink
    quantization remains visible; metre home-coordinate steps cancel before
    they reach any TF edge or geographic consumer.
    """
    def __init__(self):
        self.generation = uuid.uuid4().hex
        self.anchor = None
        self.offsets = StateHistory()
        self.corrections = StateHistory()
        self.corrections.add(0, (0., 0., 0.))
        self.latest_home_stamp = -1
        self._last_pair = None

    def update_home(self, stamp, geo, local):
        geo, local = tuple(geo), tuple(local)
        if len(geo) != 3 or len(local) != 3 or not all(
                math.isfinite(v) for v in geo + local):
            raise ValueError('nonfinite home position')
        if not -90 <= geo[0] <= 90 or not -180 <= geo[1] <= 180:
            raise ValueError('invalid home geography')
        if self.anchor is None:
            self.anchor = geo
        pair = geo + local
        if self.offsets.contains(stamp):
            # Repeated HOME_POSITION retains its original event stamp.
            return False
        changed = pair != self._last_pair
        if stamp > self.latest_home_stamp:
            self.latest_home_stamp = stamp
            self._last_pair = pair
        projected = pm.geodetic2enu(*geo, *self.anchor, deg=True)
        self.offsets.add(stamp, tuple(float(a - b) for a, b in zip(projected, local)))
        return changed or stamp < self.latest_home_stamp

    def update_correction(self, stamp, xyz):
        xyz = tuple(xyz)
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise ValueError('nonfinite survey correction')
        self.corrections.add(stamp, xyz)

    def state(self, stamp, raw_frame, home_frame, ekf_frame):
        offset = self.offsets.at(stamp)
        if self.anchor is None or offset is None:
            return None
        correction = self.corrections.at(stamp)
        return ReferenceState(self.generation, stamp, self.anchor, offset,
                              correction, raw_frame, home_frame, ekf_frame)

    def ingest(self, state):
        """Adopt an onboard canonical state for a ground reconstruction."""
        if self.anchor is not None and state.generation != self.generation:
            if state.stamp <= self.latest_home_stamp:
                return False
            self.offsets = StateHistory()
            self.corrections = StateHistory()
            self.latest_home_stamp = -1
        self.generation = state.generation
        self.anchor = state.anchor
        self.offsets.add(state.stamp, state.ekf_offset)
        self.corrections.add(state.stamp, state.correction)
        self.latest_home_stamp = max(self.latest_home_stamp, state.stamp)
        return True
