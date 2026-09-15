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
