"""Footprint display placement without a full TF listener."""
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from mavinsight.localization_reference import LocalizationReference, ReferenceState
from models.footprint_viz import FootprintViz
from models.graph_member import GraphMember
from models.vehicle import Vehicle

FRAMES = ('uas3_home_uncorrected', 'uas3_home_position', 'uas3_ekf_origin')
ANCHOR = (38., -76., 20.)


def state(correction=(1., -2., 3.)):
    return ReferenceState('generation', 10, ANCHOR, (2., 0., 0.), correction, *FRAMES)


def display(monkeypatch, frame):
    parameters = dict(camera_fov_topic='/uas3/camera_fov', localization_frame=frame,
                      local_fix_topic=('/fiducial/fix' if frame == 'fiducial'
                                       else '/uas3/home_position/fix'))
    subscriptions = []
    monkeypatch.setattr(GraphMember, '__init__', lambda self: None)
    monkeypatch.setattr(FootprintViz, 'get_logger', lambda self: SimpleNamespace(
        info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None))
    monkeypatch.setattr(FootprintViz, 'has_parameter', lambda self, name: name in parameters)
    monkeypatch.setattr(FootprintViz, 'get_parameter',
                        lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(FootprintViz, 'default_parameter_warning', lambda *a: None)
    monkeypatch.setattr(FootprintViz, 'DISPLAY_NAME', 'test', raising=False)
    monkeypatch.setattr(FootprintViz, 'create_subscription',
                        lambda self, kind, topic, cb, qos: subscriptions.append((kind, topic, cb, qos)))
    monkeypatch.setattr(FootprintViz, 'create_publisher',
                        lambda *a: SimpleNamespace(publish=lambda msg: None))
    return FootprintViz(), subscriptions


def test_home_display_waits_for_retained_state_and_has_no_tf_listener(monkeypatch):
    node, subscriptions = display(monkeypatch, FRAMES[1])
    assert node.local_fix is node.correction is None
    assert not hasattr(node, 'survey')
    kind, topic, callback, qos = next(s for s in subscriptions if s[0] is String)
    assert topic == '/uas3/localization/reference'
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.depth == 1
    callback(String(data=state().encode()))
    assert (node.local_fix.latitude, node.local_fix.longitude, node.local_fix.altitude) == ANCHOR
    assert node.correction == pytest.approx((1., -2., 3.))
    callback(String(data=state((0., 0., 0.)).encode()))
    assert node.correction == pytest.approx((0., 0., 0.))


def test_root_display_needs_only_geographic_fix(monkeypatch):
    node, subscriptions = display(monkeypatch, 'fiducial')
    assert not any(kind is String for kind, *_ in subscriptions)
    assert not hasattr(node, 'survey')
    assert node.correction == pytest.approx((0., 0., 0.))
    node.local_fix_cb(NavSatFix(latitude=38., longitude=-76., altitude=20.))
    assert node.local_fix.altitude == 20.


def test_invalid_or_wrong_vehicle_state_cannot_replace_placement(monkeypatch):
    node, _ = display(monkeypatch, FRAMES[1])
    node.reference_cb(String(data=state().encode()))
    original = node.local_fix
    for data in ('invalid json', state().__class__(
            'other', 11, ANCHOR, (0., 0., 0.), (9., 9., 9.), 'raw', 'home', 'ekf').encode()):
        node.reference_cb(String(data=data))
        assert node.local_fix is original
        assert node.correction == pytest.approx((1., -2., 3.))


def test_outline_subtracts_correction_and_map_works_before_reference(monkeypatch):
    node, _ = display(monkeypatch, FRAMES[1])
    maps, markers = [], []
    node.publish_geojson = maps.append
    node.marker_pub = SimpleNamespace(publish=markers.append)
    msg = SimpleNamespace(fov_polygon=[NavSatFix()], header=Header())
    node.fov_cb(msg)
    assert maps == [msg] and not markers
    node.reference_cb(String(data=state().encode()))
    monkeypatch.setattr('models.footprint_viz.lla_2_enu', lambda *a, **k: (10., 20., 30.))
    node.fov_cb(msg)
    assert markers[0].markers[0].points[0] == Point(x=9., y=22., z=27.)


def test_periodic_refresh_republishes_latest_without_new_history_events():
    reference = LocalizationReference()
    reference.update_home(10, ANCHOR, (0., 0., 0.))
    reference.update_correction(20, (1., 2., 3.))
    states, events = [], []
    node = SimpleNamespace(
        localization_reference=reference, _correction_activation=20,
        UNCORRECTED_HOME_FRAME=FRAMES[0], HOME_FRAME=FRAMES[1], EKF_FRAME=FRAMES[2],
        _fiducial_lla=None,
        reference_pub=SimpleNamespace(publish=lambda msg: states.append(ReferenceState.decode(msg.data))),
        reference_events_pub=SimpleNamespace(publish=events.append),
        home_fix_pub=SimpleNamespace(publish=lambda msg: None),
        ekf_fix_pub=SimpleNamespace(publish=lambda msg: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Header().stamp)))
    node._reference_state = lambda stamp: reference.state(0, *FRAMES)
    node._publish_latest_reference = lambda: Vehicle._publish_latest_reference(node)
    Vehicle._publish_reference_fixes(node)
    Vehicle._publish_reference_fixes(node)
    assert len(states) == 2 and not events
    assert states[0] == states[1]
    assert states[0].stamp == 20
    assert states[0].correction == (1., 2., 3.)
