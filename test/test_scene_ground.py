"""Ground placement shared by the terrain and building visualizations."""

import numpy as np
import pytest
import pymap3d as pm
from types import SimpleNamespace
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from mavinsight.localization_reference import ReferenceState
from models.scene_ground import (GROUND_CLEARANCE_M, SceneGround, TerrainSurface, as_fix,
                                 grounded_scene_offset)

ORIGIN = (38.313294, -76.552210, 10.5)


def fix_at(east: float, north: float, altitude: float):
    latitude, longitude, _ = pm.enu2geodetic(
        east, north, 0.0, *ORIGIN, deg=True)
    return as_fix((latitude, longitude, altitude))


def sloped_surface():
    return TerrainSurface(200.0, 2, [
        [-2.0, 0.0, 2.0],
        [-2.0, 0.0, 2.0],
        [-2.0, 0.0, 2.0],
    ])


def test_reported_home_altitude_places_scene_at_real_vertical_separation():
    anchor = as_fix(ORIGIN)
    low = grounded_scene_offset(fix_at(25.0, 0.0, 9.0), anchor,
                                np.zeros(3), sloped_surface())
    high = grounded_scene_offset(fix_at(25.0, 0.0, 49.0), anchor,
                                 np.zeros(3), sloped_surface())
    assert low[:2] == pytest.approx(high[:2], abs=1e-3)
    assert low[2] - high[2] == pytest.approx(40.0, abs=1e-3)
    assert low[2] == pytest.approx(1.5 - GROUND_CLEARANCE_M, abs=1e-3)


def test_vertical_fiducial_correction_is_applied_to_geodetic_scene_offset():
    anchor = as_fix(ORIGIN)
    offset = grounded_scene_offset(
        fix_at(0.0, 0.0, 40.0), anchor, (0.0, 0.0, 1.75),
        sloped_surface())
    assert offset[2] == pytest.approx(10.5 - 40.0 - GROUND_CLEARANCE_M - 1.75,
                                      abs=1e-3)


FRAMES = ('uas3_home_uncorrected', 'uas3_home_position', 'uas3_ekf_origin')


def reference(correction=(1., -2., 3.), anchor=ORIGIN):
    return ReferenceState('generation', 10, anchor, (2., 0., 0.), correction, *FRAMES)


def scene_ground(monkeypatch, frame, **parameters):
    subscriptions = []
    node = SimpleNamespace(
        has_parameter=lambda name: name in parameters,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None),
        create_subscription=lambda *args: subscriptions.append(args))
    def unexpected_listener(*args, **kwargs):
        pytest.fail('scene placement must not construct a TF listener')
    monkeypatch.setattr('models.scene_ground.FrameSurvey', unexpected_listener)
    monkeypatch.setattr('models.scene_ground.TransformListener', unexpected_listener)
    return SceneGround(node, frame, '/uas3/home_position/fix', 33.), subscriptions


def test_vehicle_scene_waits_for_atomic_retained_reference(monkeypatch):
    ground, subscriptions = scene_ground(monkeypatch, FRAMES[1])
    assert ground.offset(ORIGIN) is None
    assert not hasattr(ground, 'survey')
    kind, topic, callback, qos = subscriptions[0]
    assert kind is String
    assert topic == '/uas3/localization/reference'
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.depth == 1
    callback(String(data=reference().encode()))
    expected = grounded_scene_offset(as_fix(ORIGIN), ground.anchor(ORIGIN),
                                     (1., -2., 3.), None)
    assert ground.offset(ORIGIN) == pytest.approx(expected)


def test_survey_update_reset_and_anchor_change_move_scene(monkeypatch):
    ground, _ = scene_ground(monkeypatch, FRAMES[1])
    ground._reference_cb(String(data=reference().encode()))
    initial = ground.offset(ORIGIN)
    ground.drawn_at = initial
    # A periodic repeat must not trigger a scene rebuild.
    ground._reference_cb(String(data=reference().encode()))
    assert not ground.moved(ground.offset(ORIGIN))
    ground._reference_cb(String(data=reference((4., 2., 1.)).encode()))
    changed = ground.offset(ORIGIN)
    assert changed - initial == pytest.approx((-3., -4., 2.))
    assert ground.moved(changed)
    ground._reference_cb(String(data=reference((0., 0., 0.)).encode()))
    reset = ground.offset(ORIGIN)
    assert reset - initial == pytest.approx((1., -2., 3.))
    ground._reference_cb(String(data=reference((0., 0., 0.),
        (ORIGIN[0], ORIGIN[1], ORIGIN[2] + 5.)).encode()))
    assert ground.offset(ORIGIN)[2] == pytest.approx(reset[2] - 5.)


def test_fiducial_scene_uses_fix_without_survey_subscription(monkeypatch):
    ground, subscriptions = scene_ground(monkeypatch, 'fiducial')
    assert len(subscriptions) == 1
    assert subscriptions[0][0] is NavSatFix
    assert ground.offset(ORIGIN) is None
    subscriptions[0][2](as_fix(ORIGIN))
    assert ground.offset(ORIGIN) == pytest.approx(
        (0., 0., 33. - GROUND_CLEARANCE_M), abs=1e-8)


def test_reference_topic_override_and_raw_frame_ignore_correction(monkeypatch):
    ground, subscriptions = scene_ground(monkeypatch, FRAMES[0],
        localization_reference_topic='/custom/reference')
    assert subscriptions[0][1] == '/custom/reference'
    ground._reference_cb(String(data=reference().encode()))
    assert ground.correction == (0., 0., 0.)


def test_invalid_and_unrelated_reference_leave_placement_intact(monkeypatch):
    ground, _ = scene_ground(monkeypatch, FRAMES[1])
    ground._reference_cb(String(data=reference().encode()))
    before = ground.offset(ORIGIN)
    unrelated = ReferenceState('other', 20, ORIGIN, (0., 0., 0.), (9., 9., 9.),
                               'other_raw', 'other_home', 'other_ekf')
    for data in ('invalid json', unrelated.encode()):
        ground._reference_cb(String(data=data))
        assert ground.offset(ORIGIN) == pytest.approx(before)
