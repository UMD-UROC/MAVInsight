from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation as R
from cdcl_umd_msgs.msg import FiducialCalibration
from mavros_msgs.msg import GimbalDeviceAttitudeStatus

from models.sensor import Gimbal
from models.vehicle import Vehicle


def packet():
    p=FiducialCalibration();p.gimbal_frame='uas3_gimbal_frame'
    p.header.frame_id='uas3_home_uncorrected'
    p.translation.x=1.;p.translation.y=-2.;p.translation.z=3.
    q=R.from_rotvec([0.,.02,-.01]).as_quat()
    for axis,value in zip(('x','y','z','w'),q):setattr(p.sensor_rotation,axis,float(value))
    return p


def test_vehicle_keeps_sensor_rotation_out_of_navigation_edge():
    received=[]
    v=SimpleNamespace(UNCORRECTED_HOME_FRAME='uas3_home_uncorrected',
        HOME_FRAME='uas3_home_position',update_fiducial=received.append)
    Vehicle.update_calibration(v,packet())
    assert len(received)==1
    t=received[0].transform
    assert (t.translation.x,t.translation.y,t.translation.z)==(1.,-2.,3.)
    assert (t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)==(0.,0.,0.,1.)


def test_gimbal_preserves_uncorrected_frame_and_postrotates_only_once():
    received=[]
    g=SimpleNamespace(FRAME_NAME='uas3_gimbal_frame',GIMBAL_REF_FRAME_NAME='ref',
        IGNORE_REPORTED_YAW=False,tf_broadcaster=SimpleNamespace(sendTransform=received.append))
    Gimbal.update_calibration(g,packet())
    msg=GimbalDeviceAttitudeStatus();msg.q.w=1.
    Gimbal.publish_orientation(g,msg)
    nominal,corrected=received[0]
    assert nominal.child_frame_id=='uas3_gimbal_frame_uncorrected'
    nq=nominal.transform.rotation
    np.testing.assert_allclose(R.from_quat([nq.x,nq.y,nq.z,nq.w]).as_matrix(),np.eye(3))
    q=corrected.transform.rotation
    np.testing.assert_allclose(R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),
                               g._calibration_rotation.as_matrix())
    Gimbal.publish_orientation(g,msg)
    q2=received[1][1].transform.rotation
    np.testing.assert_allclose(R.from_quat([q2.x,q2.y,q2.z,q2.w]).as_matrix(),
                               g._calibration_rotation.as_matrix())


def test_late_attitude_uses_calibration_active_at_measurement_time():
    received=[]
    g=SimpleNamespace(FRAME_NAME='uas3_gimbal_frame', GIMBAL_REF_FRAME_NAME='ref',
        IGNORE_REPORTED_YAW=False, tf_broadcaster=SimpleNamespace(sendTransform=received.append))
    p=packet(); p.header.stamp.sec=3
    Gimbal.update_calibration(g,p)
    msg=GimbalDeviceAttitudeStatus(); msg.q.w=1.; msg.header.stamp.sec=2
    Gimbal.publish_orientation(g,msg)
    q=received[-1][1].transform.rotation
    np.testing.assert_allclose(R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.eye(3))
    msg.header.stamp.sec=4
    Gimbal.publish_orientation(g,msg)
    q=received[-1][1].transform.rotation
    np.testing.assert_allclose(R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),g._calibration_rotation.as_matrix())
