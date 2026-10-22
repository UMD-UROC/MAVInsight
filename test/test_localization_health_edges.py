"""Only the active localization branches contribute required dynamic edges."""
import pytest

from mavinsight.frame_config import frame_configs, localization_dynamic_edges


@pytest.mark.parametrize('model', ['v2', 'v3'])
@pytest.mark.parametrize('encoder,source', [(False, 'fused'), (True, 'fused'), (True, 'encoder')])
@pytest.mark.parametrize('camera', ['rgb', 'thermal'])
def test_active_localization_edges(model, encoder, source, camera):
    members = frame_configs(3, model, camera=camera, gimbal_encoder=encoder,
                            gimbal_pose_source=source)
    frames = dict(localization_fiducial_frame='fiducial',
                  localization_origin_frame='uas3_home_position',
                  localization_uncorrected_origin_frame='uas3_home_uncorrected',
                  drone_frame='uas3_base_link', camera_frame='uas3_rgb_offset',
                  gimbal_frame='uas3_gimbal_frame' + ('_encoder' if source == 'encoder' else ''),
                  rangefinder_frame='uas3_rangefinder_frame')
    expected = {
        ('fiducial', 'uas3_home_uncorrected'),
        ('uas3_home_uncorrected', 'uas3_home_position'),
        ('uas3_home_position', 'uas3_ekf_origin'),
        ('uas3_ekf_origin', 'uas3_base_link'),
        ('uas3_rangefinder_frame_offset', 'uas3_rangefinder_frame'),
    }
    if source == 'encoder':
        expected.add(('uas3_gimbal_frame_offset', 'uas3_gimbal_frame_encoder'))
    else:
        expected.update({('uas3_gimbal_frame_offset', 'uas3_gimbal_frame_ref'),
                         ('uas3_gimbal_frame_ref', 'uas3_gimbal_frame')})
    assert set(localization_dynamic_edges(members, frames)) == expected


def test_unknown_localization_frame_fails_instead_of_silently_omitting_a_path():
    with pytest.raises(ValueError, match='No configured localization TF path'):
        localization_dynamic_edges([], dict(localization_fiducial_frame='map',
                                            localization_origin_frame='missing'))
