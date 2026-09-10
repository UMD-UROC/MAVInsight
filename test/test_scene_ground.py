"""Ground placement shared by the terrain and building visualizations."""

import numpy as np
import pytest
import pymap3d as pm

from models.scene_ground import (GROUND_CLEARANCE_M, TerrainSurface, as_fix,
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


def test_reported_home_altitude_does_not_move_the_scene_ground():
    anchor = as_fix(ORIGIN)
    low = grounded_scene_offset(fix_at(25.0, 0.0, 9.0), anchor,
                                np.zeros(3), sloped_surface())
    high = grounded_scene_offset(fix_at(25.0, 0.0, 49.0), anchor,
                                 np.zeros(3), sloped_surface())
    assert low == pytest.approx(high, abs=1e-3)
    assert low[2] + sloped_surface().height(-low[0], -low[1]) \
        == pytest.approx(-GROUND_CLEARANCE_M)


def test_vertical_fiducial_correction_overrides_grounded_home():
    anchor = as_fix(ORIGIN)
    offset = grounded_scene_offset(
        fix_at(0.0, 0.0, 40.0), anchor, (0.0, 0.0, 1.75),
        sloped_surface())
    assert offset[2] == pytest.approx(-GROUND_CLEARANCE_M - 1.75)
