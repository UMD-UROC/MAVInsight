"""Regression coverage for paired home changes and reference time ordering."""
import numpy as np
import pymap3d as pm
import pytest
from mavinsight.localization_reference import LocalizationReference, ReferenceState, StateHistory

FRAMES = ('uas3_home_uncorrected', 'uas3_home_position', 'uas3_ekf_origin')
ANCHOR = (38.31328, -76.55221, 20.)


def test_recorded_altitude_steps_cancel_including_next_takeoff():
    reference = LocalizationReference()
    reference.update_home(1, ANCHOR, (0., 0., -5.))
    reference.update_correction(1, (1.2, -.5, 2.))
    original = reference.state(1, *FRAMES)
    cumulative = 0.
    # Recorded NED increments converted to ROS ENU; includes the exact
    # inverse of the previous flight's accumulated correction at takeoff.
    for time, delta in enumerate((1.0151519775390625, 1.0044631958007812,
                                 1.0490264892578125, -3.0686416625976562,
                                 1.0100021362304688), start=2):
        cumulative -= delta
        geo = (ANCHOR[0], ANCHOR[1], ANCHOR[2] + cumulative)
        reference.update_home(time, geo, (0., 0., -5. + cumulative))
        current = reference.state(time, *FRAMES)
        assert current.anchor == original.anchor
        assert current.correction == original.correction
        assert current.ekf_offset == pytest.approx(original.ekf_offset, abs=1e-8)
        assert current.frame_anchor(FRAMES[1]) == original.frame_anchor(FRAMES[1])


def test_horizontal_home_reset_also_cancels_before_frame_consumers():
    reference = LocalizationReference()
    reference.update_home(1, ANCHOR, (2., 3., -4.))
    moved = tuple(pm.enu2geodetic(10., -15., 2., *ANCHOR, deg=True))
    reference.update_home(2, moved, (12., -12., -2.))
    assert reference.state(2, *FRAMES).ekf_offset == pytest.approx((-2., -3., 4.), abs=1e-8)


def test_genuine_composed_anchor_change_is_preserved():
    reference = LocalizationReference()
    reference.update_home(1, ANCHOR, (0., 0., 0.))
    reference.update_home(2, (ANCHOR[0], ANCHOR[1], ANCHOR[2] + 3.), (0., 0., 0.))
    assert reference.state(2, *FRAMES).ekf_offset[2] == pytest.approx(3.)
    assert reference.state(1, *FRAMES).ekf_offset[2] == pytest.approx(0.)


def test_repeated_and_out_of_order_home_cannot_roll_back_current():
    reference = LocalizationReference()
    assert reference.update_home(10, ANCHOR, (0., 0., 0.))
    assert reference.update_home(20, ANCHOR, (0., 0., 3.))
    assert not reference.update_home(10, ANCHOR, (0., 0., 0.))
    assert not reference.update_home(20, ANCHOR, (0., 0., 3.))
    assert reference.state(30, *FRAMES).ekf_offset == (0., 0., -3.)


def test_calibration_history_is_stepwise_and_never_applies_future_state():
    reference = LocalizationReference()
    reference.update_home(10, ANCHOR, (0., 0., 0.))
    reference.update_correction(30, (1., 2., 3.))
    reference.update_correction(20, (-1., -2., -3.))
    assert reference.state(9, *FRAMES) is None
    assert reference.state(19, *FRAMES).correction == (0., 0., 0.)
    assert reference.state(20, *FRAMES).correction == (-1., -2., -3.)
    assert reference.state(29, *FRAMES).correction == (-1., -2., -3.)
    assert reference.state(30, *FRAMES).correction == (1., 2., 3.)


def test_reference_wire_snapshot_round_trip_and_validation():
    reference = LocalizationReference()
    reference.update_home(10, ANCHOR, (1., 2., 3.))
    state = reference.state(10, *FRAMES)
    assert ReferenceState.decode(state.encode()) == state
    with pytest.raises(ValueError):
        reference.update_home(20, (38., -76., np.nan), (0., 0., 0.))
    assert reference.state(20, *FRAMES) == reference.state(10, *FRAMES).__class__(
        state.generation, 20, state.anchor, state.ekf_offset, state.correction, *FRAMES)


def test_history_retains_bounded_predecessors_and_latest_on_late_insert():
    history = StateHistory(limit=3)
    for time in (10, 30, 20, 40):
        history.add(time, time)
    assert history.at(10) is None
    assert history.at(25) == 20
    assert history.at() == 40


def test_late_home_event_repairs_history_without_replacing_latest():
    reference = LocalizationReference()
    reference.update_home(10, ANCHOR, (0., 0., 0.))
    reference.update_home(30, ANCHOR, (0., 0., 3.))
    reference.update_home(20, ANCHOR, (0., 0., 2.))
    assert reference.state(25, *FRAMES).ekf_offset[2] == -2.
    assert reference.state(35, *FRAMES).ekf_offset[2] == -3.
    assert reference.latest_home_stamp == 30


def test_ground_adopts_air_anchor_instead_of_choosing_late_home():
    air = LocalizationReference()
    air.update_home(10, ANCHOR, (0., 0., -5.))
    air.update_home(20, (ANCHOR[0], ANCHOR[1], ANCHOR[2] - 2.), (0., 0., -7.))
    air.update_correction(20, (1., -2., .5))
    ground = LocalizationReference()
    assert ground.ingest(ReferenceState.decode(air.state(20, *FRAMES).encode()))
    assert ground.state(20, *FRAMES) == air.state(20, *FRAMES)
    # A delayed snapshot from the same generation repairs history only.
    ground.ingest(air.state(10, *FRAMES))
    assert ground.state(30, *FRAMES).correction == (1., -2., .5)
    assert ground.state(15, *FRAMES).correction == (0., 0., 0.)


@pytest.mark.parametrize('fix,expected', [
    ('home_position/fix', 'localization/reference'),
    ('/uas3/home_position/fix', '/uas3/localization/reference'),
    ('/uas3/ekf_origin/fix', '/uas3/localization/reference'),
])
def test_reference_topic_is_vehicle_scoped_for_relative_and_absolute_fixes(fix, expected):
    from mavinsight.localization_reference import reference_topic
    assert reference_topic(fix) == expected
    assert reference_topic(fix, events=True) == expected + '_events'
