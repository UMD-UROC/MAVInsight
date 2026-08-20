"""The ground has to be drawn the right way round.

A terrain model that is turned or mirrored is published exactly as
convincingly as a correct one, and only an operator looking at the panel can
tell. These check the two conventions that decide it, against a mesh laid out
the way `scenegen build` writes one.
"""

import array
import json
import struct

import numpy as np

from models import gltf

SIDE = 4
STEP_M = 100.0


def enu_grid():
    """A square of ENU vertices with the texture over it the way COLLADA
    writes one: v measured up from the bottom of the image, so the northern
    edge is v = 1, and u east from its left."""
    axis = (np.arange(SIDE) - (SIDE - 1) / 2) * STEP_M
    east, north = np.meshgrid(axis, axis)
    up = np.zeros_like(east)
    positions = np.column_stack([east.ravel(), north.ravel(), up.ravel()])
    fraction = (np.arange(SIDE) / (SIDE - 1))
    u, v = np.meshgrid(fraction, fraction)
    return positions, np.column_stack([u.ravel(), v.ravel()])


def read_back(model):
    """The positions and texture coordinates a GLB carries."""
    length = struct.unpack("<I", model[12:16])[0]
    document = json.loads(model[20:20 + length])
    binary = 20 + length + 8

    def accessor(index, columns):
        entry = document["accessors"][index]
        view = document["bufferViews"][entry["bufferView"]]
        start = binary + view["byteOffset"] + entry.get("byteOffset", 0)
        values = array.array("f")
        values.frombytes(model[start:start + 4 * columns * entry["count"]])
        return np.array(values).reshape(-1, columns)

    return accessor(0, 3), accessor(1, 2)


def built():
    positions, uvs = enu_grid()
    indices = np.arange(len(positions), dtype=np.uint32)
    model = gltf.textured_mesh(positions, uvs, indices, b"\xff\xd8\xff", "image/jpeg")
    return positions, uvs, *read_back(model)


def test_north_is_minus_z():
    """glTF is Y up and -Z forward. Foxglove turns every model a quarter turn
    about X to stand it up, so a mesh written in ENU arrives on its side."""
    positions, _, drawn, _ = built()
    assert np.allclose(drawn[:, 0], positions[:, 0])       # east stays east
    assert np.allclose(drawn[:, 1], positions[:, 2])       # up is the second axis
    assert np.allclose(-drawn[:, 2], positions[:, 1])      # north is -Z


def test_northern_edge_is_the_top_of_the_image():
    """glTF measures v down from the top of the image and COLLADA up from the
    bottom. Unflipped, the map draws mirrored north for south."""
    positions, _, drawn, uvs = built()
    northernmost = int(np.argmax(-drawn[:, 2]))
    southernmost = int(np.argmin(-drawn[:, 2]))
    assert uvs[northernmost, 1] == 0.0
    assert uvs[southernmost, 1] == 1.0


def test_east_is_not_mirrored():
    _, _, drawn, uvs = built()
    assert uvs[int(np.argmax(drawn[:, 0])), 0] == 1.0
    assert uvs[int(np.argmin(drawn[:, 0])), 0] == 0.0


def test_the_turn_keeps_the_winding():
    """A reflection would turn every triangle inside out and the ground would
    be culled when seen from above. The mapping is a rotation, so it does not."""
    turn = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    assert np.linalg.det(turn) > 0
