"""Shared georeferenced image-overlay helpers for 3D scene textures."""
import io

import numpy as np
from PIL import Image

from models.frame_utils import lla_2_enu
from models.scene_ground import as_fix


def overlay_box_enu(overlay, scene_origin_lla):
    """Return overlay bounds (west, south, east, north) in scene-centre ENU."""
    if overlay is None or scene_origin_lla is None:
        return None
    anchor = as_fix((scene_origin_lla[0], scene_origin_lla[1], 0.0))
    west, south, _ = lla_2_enu(anchor, as_fix((overlay.sw_lat, overlay.sw_lon, 0.0)))
    east, north, _ = lla_2_enu(anchor, as_fix((overlay.ne_lat, overlay.ne_lon, 0.0)))
    if not (east > west and north > south):
        return None
    return (west, south, east, north)


def enu_to_uv_affine(positions, uvs):
    """Fit the affine taking scene ENU positions to texture UV coordinates."""
    positions = np.asarray(positions).reshape(-1, 3)
    uvs = np.asarray(uvs).reshape(-1, 2)
    design = np.column_stack([np.ones(len(positions)), positions[:, 0], positions[:, 1]])
    return np.linalg.lstsq(design, uvs, rcond=None)[0].T


def texture_to_overlay(affine, texture_size, overlay_box, overlay_size):
    """PIL inverse affine mapping texture pixels into overlay pixels."""
    texture_w, texture_h = texture_size
    overlay_w, overlay_h = overlay_size
    to_uv = np.array([[1.0 / texture_w, 0.0, 0.0],
                      [0.0, -1.0 / texture_h, 1.0],
                      [0.0, 0.0, 1.0]])
    to_uv_from_enu = np.array([[affine[0, 1], affine[0, 2], affine[0, 0]],
                               [affine[1, 1], affine[1, 2], affine[1, 0]],
                               [0.0, 0.0, 1.0]])
    to_enu = np.linalg.inv(to_uv_from_enu)
    west, south, east, north = overlay_box
    across = (overlay_w - 1) / (east - west)
    down = (overlay_h - 1) / (north - south)
    to_pixel = np.array([[across, 0.0, -west * across],
                         [0.0, -down, north * down],
                         [0.0, 0.0, 1.0]])
    return tuple((to_pixel @ to_enu @ to_uv)[:2].ravel())


def composite_overlay(base, overlay_msg, scene_origin_lla, positions, uvs):
    """Drape a MosaicOverlay onto any scene texture using that geometry's UV map.

    Returns (image, box, covered_fraction). Raises OSError/ValueError when the
    encoded overlay cannot be decoded. A None box means the overlay cannot be
    georeferenced against this scene.
    """
    box = overlay_box_enu(overlay_msg, scene_origin_lla)
    if box is None:
        return base, None, 0.0
    overlay = Image.open(io.BytesIO(bytes(overlay_msg.overlay_png.data))).convert("RGBA")
    coefficients = texture_to_overlay(
        enu_to_uv_affine(positions, uvs), base.size, box, overlay.size)
    placed = overlay.transform(base.size, Image.AFFINE, coefficients,
                               resample=Image.BILINEAR)
    drawn = base.copy()
    drawn.paste(placed, (0, 0), placed)
    covered = float(np.asarray(placed)[..., 3].astype(bool).mean())
    return drawn, box, covered
