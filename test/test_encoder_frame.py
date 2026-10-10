"""Encoder joints retain body/level disagreement and the existing calibration."""
from collections import deque
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

from mavinsight.localization_reference import StateHistory
from models.gimbal_frame import encoder_rotation, reported_roll_pitch
from models.sensor import Gimbal


def test_outer_roll_rotates_the_inner_pitch_axis():
    # A pitched boresight tilts sideways when the outer roll is rotated.
    direction = encoder_rotation(np.pi/4,np.pi/4).apply([1,0,0])
    np.testing.assert_allclose(direction,[np.sqrt(.5),.5,-.5],atol=1e-12)


def test_body_level_disagreement_is_retained():
    body = R.from_euler('x',10,degrees=True)
    measured = body*encoder_rotation(0,0)
    reported_world = R.identity()
    assert np.isclose(np.rad2deg((reported_world.inv()*measured).magnitude()),10)
    # Real encoder compensation can cancel the body tilt without altering zero.
    compensated = body*encoder_rotation(0,np.deg2rad(-10))
    np.testing.assert_allclose(compensated.as_matrix(),np.eye(3),atol=1e-12)


def test_three_axis_outer_yaw_and_v2_body_yaw():
    body=R.from_euler('z',37,degrees=True)
    v2=body*encoder_rotation(0,0)
    assert np.isclose(v2.as_euler('xyz',degrees=True)[2],37)
    v3=body*encoder_rotation(np.pi/4,np.pi/4,np.pi/2)
    np.testing.assert_allclose(v3.as_matrix(),
        (body*R.from_euler('z',90,degrees=True)*R.from_euler('x',45,degrees=True)*R.from_euler('y',45,degrees=True)).as_matrix(),atol=1e-12)


def test_full_report_keeps_roll_and_removes_nonphysical_yaw():
    report = R.from_euler('xyz',[23,-45,72],degrees=True)
    np.testing.assert_allclose(reported_roll_pitch(report).as_euler('xyz',degrees=True),[23,-45,0],atol=1e-12)


def test_encoder_uses_existing_mount_and_time_specific_calibration():
    received = []
    history = StateHistory()
    history.add(0,R.identity())
    correction = R.from_euler('z',3,degrees=True)
    history.add(3_000_000_000,correction)
    g = SimpleNamespace(FRAME_NAME='uas3_gimbal_frame',PARENT_FRAME='uas3_gimbal_frame_offset',
                        _calibration_history=history,_encoder_pending=deque(),
                        tf_broadcaster=SimpleNamespace(sendTransform=received.append))
    msg = JointState()
    msg.header.frame_id = g.PARENT_FRAME
    msg.header.stamp.sec = 4
    msg.name = ['roll','pitch']  # Names, rather than slot ordering, drive TF.
    msg.position = [np.deg2rad(20),np.deg2rad(35)]
    Gimbal.publish_encoder(g,msg)
    nominal,calibrated = received[-1]
    assert nominal.header.frame_id == calibrated.header.frame_id == g.PARENT_FRAME
    q=calibrated.transform.rotation
    np.testing.assert_allclose(R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),
        (encoder_rotation(msg.position[1],msg.position[0])*correction).as_matrix())
    msg.header.stamp.sec=2
    Gimbal.publish_encoder(g,msg)
    q=received[-1][1].transform.rotation
    np.testing.assert_allclose(R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),
        encoder_rotation(msg.position[1],msg.position[0]).as_matrix())
    count=len(received)
    msg.position[0]=float('nan')
    Gimbal.publish_encoder(g,msg)
    assert len(received)==count
    # A future three-axis publisher must supply its measured yaw.
    g.ENCODER_HAS_YAW_AXIS=True
    msg.position[0]=0.0
    Gimbal.publish_encoder(g,msg)
    assert len(received)==count
    msg.name.append('yaw'); msg.position.append(np.pi/2)
    Gimbal.publish_encoder(g,msg)
    assert len(received)==count+1
