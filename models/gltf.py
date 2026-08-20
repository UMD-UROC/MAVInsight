"""One textured mesh, as a self-contained binary glTF.

Foxglove draws a triangle list and no texture, so a satellite image over
terrain has to arrive as a model. A ModelPrimitive carries the model's bytes in
the message, which is why this writes a GLB rather than a file: nothing has to
serve it and nothing has to reach a URL.

Written out rather than taken from a library because the whole of what is
needed is one mesh, one image and one material, and glTF says plainly how to
lay those out.

Callers hand over ENU and texture coordinates measured up from the bottom of
the image, which is what a scene and a COLLADA mesh are written in. Both are
turned here, because both are glTF's business rather than the caller's.
"""

import json
import struct

import numpy as np

# The chunk types a GLB holds, and the alignment every chunk is padded to.
JSON_CHUNK = 0x4E4F534A
BINARY_CHUNK = 0x004E4942
ALIGNMENT = 4

FLOAT = 5126
UNSIGNED_INT = 5125
ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963


def _as_gltf(positions, uvs):
    """ENU and v-up, in the axes and texture space glTF is defined in.

    glTF is Y up and -Z forward, so east stays east, up becomes the second
    axis and north becomes -Z. Foxglove turns every glTF model a quarter turn
    about X to stand it up in a Z up world, and this is exactly that turn
    undone: written any other way the ground arrives on its side.

    Texture space differs the same way. COLLADA and OpenGL measure v up from
    the bottom of the image, glTF down from the top, and a mesh whose v is not
    flipped draws the map mirrored north for south.

    The turn is a rotation rather than a reflection, so the triangles keep the
    winding the caller gave them.
    """
    east, north, up = positions[:, 0], positions[:, 1], positions[:, 2]
    return (np.column_stack([east, up, -north]),
            np.column_stack([uvs[:, 0], 1.0 - uvs[:, 1]]))


def _padded(data: bytes, filler: bytes) -> bytes:
    remainder = len(data) % ALIGNMENT
    return data if remainder == 0 else data + filler * (ALIGNMENT - remainder)


def textured_mesh(positions, uvs, indices, image: bytes, media_type: str) -> bytes:
    """A GLB of one mesh with one image over it.

    `positions` is (n, 3) ENU and `uvs` is (n, 2) measured up from the bottom
    of the image, both as the caller holds them. `indices` is a flat uint32
    array of triangle corners.
    """
    positions, uvs = _as_gltf(positions, uvs)
    position_bytes = positions.astype("<f4").tobytes()
    uv_bytes = uvs.astype("<f4").tobytes()
    index_bytes = indices.astype("<u4").tobytes()

    buffer = b""
    views = []
    for payload, target in ((position_bytes, ARRAY_BUFFER),
                            (uv_bytes, ARRAY_BUFFER),
                            (index_bytes, ELEMENT_ARRAY_BUFFER),
                            (image, None)):
        buffer = _padded(buffer, b"\x00")
        view = {"buffer": 0, "byteOffset": len(buffer), "byteLength": len(payload)}
        if target is not None:
            view["target"] = target
        views.append(view)
        buffer += payload

    lowest = positions.min(axis=0).tolist()
    highest = positions.max(axis=0).tolist()
    document = {
        "asset": {"version": "2.0", "generator": "MAVInsight"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{
            "attributes": {"POSITION": 0, "TEXCOORD_0": 1},
            "indices": 2,
            "material": 0,
        }]}],
        # Unlit, so the ground reads as a map rather than as a surface the
        # panel's own lighting has decided the brightness of.
        "extensionsUsed": ["KHR_materials_unlit"],
        "materials": [{
            "pbrMetallicRoughness": {
                "baseColorTexture": {"index": 0},
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0,
            },
            "extensions": {"KHR_materials_unlit": {}},
        }],
        "textures": [{"source": 0, "sampler": 0}],
        "samplers": [{}],
        "images": [{"bufferView": 3, "mimeType": media_type}],
        "accessors": [
            {"bufferView": 0, "componentType": FLOAT, "count": len(positions),
             "type": "VEC3", "min": lowest, "max": highest},
            {"bufferView": 1, "componentType": FLOAT, "count": len(uvs),
             "type": "VEC2"},
            {"bufferView": 2, "componentType": UNSIGNED_INT, "count": len(indices),
             "type": "SCALAR"},
        ],
        "bufferViews": views,
        "buffers": [{"byteLength": len(buffer)}],
    }

    json_chunk = _padded(json.dumps(document, separators=(",", ":")).encode(), b" ")
    binary_chunk = _padded(buffer, b"\x00")
    length = 12 + 8 + len(json_chunk) + 8 + len(binary_chunk)
    return b"".join([
        struct.pack("<4sII", b"glTF", 2, length),
        struct.pack("<II", len(json_chunk), JSON_CHUNK), json_chunk,
        struct.pack("<II", len(binary_chunk), BINARY_CHUNK), binary_chunk,
    ])
