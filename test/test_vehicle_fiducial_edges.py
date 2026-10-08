"""Raw GPS placement and fiducial correction remain separate TF edges."""

import pymap3d as pm

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Vector3
from sensor_msgs.msg import NavSatFix

from models.vehicle import Vehicle


class Clock:
    def now(self):
        return self

    def to_msg(self):
        return Time(sec=10)


class VehicleShell:
    def __init__(self, home, fiducial, correction):
        self._home_lla = home
        self._fiducial_lla = fiducial
        self._fiducial_correction = Vector3(
            x=correction[0], y=correction[1], z=correction[2])
        self.raw_home_t = TransformStamped()
        self.raw_home_t.header.frame_id = 'fiducial'
        self.raw_home_t.child_frame_id = 'uas4_home_uncorrected'
        self.correction_t = TransformStamped()
        self.correction_t.header.frame_id = 'uas4_home_uncorrected'
        self.correction_t.child_frame_id = 'uas4_home_position'
        self.publish_fiducial_edge = True

    def get_clock(self):
        return Clock()


def test_vehicle_composes_raw_home_and_correction_as_separate_edges():
    home = NavSatFix(latitude=38.0001, longitude=-75.9998, altitude=22.0)
    fiducial = [38.0, -76.0, 12.0]
    correction = (-1.25, 0.75, 2.5)
    vehicle = VehicleShell(home, fiducial, correction)

    Vehicle._compose_fiducial_edges(vehicle)

    expected_raw = pm.geodetic2enu(
        home.latitude, home.longitude, home.altitude, *fiducial, deg=True)
    raw_edge, correction_edge = Vehicle._root_tfs(vehicle, Time(sec=20))
    raw = raw_edge.transform.translation
    survey = correction_edge.transform.translation
    assert raw_edge.header.frame_id == 'fiducial'
    assert raw_edge.child_frame_id == 'uas4_home_uncorrected'
    assert correction_edge.header.frame_id == 'uas4_home_uncorrected'
    assert correction_edge.child_frame_id == 'uas4_home_position'
    assert (raw.x, raw.y, raw.z) == expected_raw
    assert (survey.x, survey.y, survey.z) == correction
    assert vehicle.raw_home_t.transform.rotation.w == 1.0
    assert vehicle.correction_t.transform.rotation.w == 1.0
    assert raw_edge.header.stamp.sec == 20
    assert correction_edge.header.stamp.sec == 20


def test_vehicle_home_callback_keeps_application_anchor_and_complete_chain(monkeypatch):
    from copy import deepcopy
    from types import SimpleNamespace
    import pytest
    from mavros_msgs.msg import HomePosition
    from mavinsight.localization_reference import LocalizationReference, ReferenceState

    v = Vehicle.__new__(Vehicle)
    monkeypatch.setattr(Vehicle, 'get_clock', lambda self: Clock())
    v.HOME_FRAME = 'uas4_home_position'
    v.UNCORRECTED_HOME_FRAME = 'uas4_home_uncorrected'
    v.EKF_FRAME = 'uas4_ekf_origin'
    v.FIDUCIAL_FRAME = 'fiducial'
    v._fiducial_lla = [38., -76., 12.]
    v._fiducial_correction = Vector3(x=1., y=-2., z=.5)
    v.raw_home_t = TransformStamped()
    v.correction_t = TransformStamped()
    v.publish_fiducial_edge = True
    v.external_reference_topic = ''
    v.localization_reference = LocalizationReference()
    v.localization_reference.update_correction(0, (1., -2., .5))
    sent, states, fixes = [], [], []
    v.tf_broadcaster = SimpleNamespace(sendTransform=lambda x: sent.append(deepcopy(x)))
    v.home_fix_pub = SimpleNamespace(publish=lambda x: fixes.append(deepcopy(x)))
    v.reference_pub = SimpleNamespace(publish=lambda x: states.append(ReferenceState.decode(x.data)))
    v.reference_events_pub = v._fiducial_fix_pub = v.ekf_fix_pub = SimpleNamespace(publish=lambda x: None)
    msg = HomePosition()
    msg.header.stamp.sec = 1
    msg.geo.latitude, msg.geo.longitude, msg.geo.altitude = 38.0001, -75.9998, 22.
    msg.position.z = -5.
    v.home_cb(msg)
    msg.header.stamp.sec = 2
    msg.geo.altitude -= 1.
    msg.position.z -= 1.
    v.home_cb(msg)
    assert fixes[-1].altitude == fixes[0].altitude == 22.
    assert states[-1].ekf_offset == pytest.approx(states[0].ekf_offset, abs=1e-8)
    for edges in sent:
        assert len(edges) == 3
        assert len({(e.header.stamp.sec, e.header.stamp.nanosec) for e in edges}) == 1
    assert sent[-1][-1].transform.translation.z == pytest.approx(5.)
