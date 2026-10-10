"""One selected pose carries both camera and laser, with telemetry-free fallback."""
import importlib.util
from pathlib import Path

import pytest
from models.gimbal_frame import resolve_gimbal_pose_source


@pytest.mark.parametrize('requested,available,expected',[
    ('auto',True,'encoder'),('auto',False,'fused'),('fused',True,'fused'),('encoder',True,'encoder')])
def test_pose_selection(requested,available,expected):
    assert resolve_gimbal_pose_source(requested,available)==expected


def test_explicit_encoder_requires_telemetry():
    with pytest.raises(ValueError):
        resolve_gimbal_pose_source('encoder',False)


@pytest.mark.parametrize('model',['v2','v3'])
@pytest.mark.parametrize('enabled,source,parent',[(True,'auto','_encoder'),(True,'fused',''),(False,'auto','')])
def test_shared_builder_reparents_both_sensor_leaves(model,enabled,source,parent):
    path=Path(__file__).resolve().parents[1]/'launch'/'launch_sim.launch.py'
    spec=importlib.util.spec_from_file_location('pose_source_launch',path)
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    configs=launch.frame_configs(3,model,gimbal_encoder=enabled,gimbal_pose_source=source)
    leaves=[c for _,c in configs if c.get('sensor_type') in ('camera','rangefinder')
            and c['frame_name'] in ('uas3_rgb','uas3_rangefinder_frame')]
    assert len(leaves)==2
    assert all(c['parent_frame']=='uas3_gimbal_frame'+parent for c in leaves)
