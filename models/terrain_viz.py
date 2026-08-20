"""Drape the scene's satellite image over its terrain in the 3D panel.

`scenegen build` writes the terrain twice: as `worlds/<scene>_surface.json`,
which `umd_uas/terrain.py` meets a detection ray with, and as a COLLADA mesh
with the satellite image over it, which Gazebo renders. The panel held neither,
so an operator saw markers floating over nothing and had to judge where a
target was from its numbers.

The mesh is what this reads, because it already carries the texture
coordinates: the imagery is a slippy tile mosaic and mapping a point into it
needs the georeference the scene no longer ships. Its vertices are the same
heights the surface file holds, so what is drawn here is the surface a ray is
localized against.

The image goes over the mesh rather than onto its vertices. Foxglove draws a
triangle list and no texture, so the ground arrives as a model instead: a
SceneUpdate carries a ModelPrimitive, and a ModelPrimitive carries the model's
bytes, so nothing has to serve a file and nothing has to reach a URL. The
detail is then the image's rather than the mesh's, which over a 600 m scene is
half a metre a pixel instead of five.

The mesh measures every coordinate from the scene centre and the markers go in
a frame centred on the vehicle's home position, the same two points
buildings_viz reconciles. `models/scene_ground.py` works out where one sits in
the other, and both scene layers read it from there, so the ground and the
buildings on it can never stand at different heights.

That offset is watched rather than read once. PX4 moves the home position when
the vehicle arms, and `umd_uas/footprint.py` casts its rays against the fix as
it is now, so a terrain placed at startup and held sinks or rises under the
footprint and the live view that lie on it. The model does not change when the
scene moves, so the bytes are held and only the pose goes out again.

Nothing goes out before the first home fix. Terrain at the wrong offset looks
correct and is wrong, which is worse than an empty panel.

Subscribes
    <local_fix_topic>       sensor_msgs/NavSatFix, the WGS84 position of
                            <reference_frame>
Publishes
    <terrain_viz_topic>     foxglove_msgs/SceneUpdate, latched
"""

# python imports
import io
import json
import math
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
from PIL import Image

# ROS2 imports
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# ROS2 message imports
from foxglove_msgs.msg import Color, ModelPrimitive, SceneEntity, SceneUpdate
from geometry_msgs.msg import Quaternion, Vector3

# MAVInsight imports
from models import gltf
from models.graph_member import GraphMember
from models.scene_ground import SceneGround

COLLADA = "{http://www.collada.org/2005/11/COLLADASchema}"
ENTITY_ID = "scene_terrain"
# How often the mesh file and the ground under it are looked at again. Both
# checks are a stat call and a subtraction, so this is what decides how long an
# operator waits after a home update or a scene rebuild.
RELOAD_CHECK_S = 5.0

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)


def param(node, name, default):
    """A parameter's value, or its default with the base class's warning. The
    base declares from the overrides, so declaring again raises."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    node.default_parameter_warning(name)
    return default


def float_array(root, ending):
    """One named float array out of a COLLADA source."""
    for source in root.iter(f"{COLLADA}source"):
        if source.get("id", "").endswith(ending):
            values = source.find(f"{COLLADA}float_array")
            return np.fromstring(values.text, sep=" ")
    raise ValueError(f"the mesh carries no {ending} array")


def grid_side(vertices: int) -> int:
    """A terrain mesh is a square grid, so its side is the root of its vertex
    count. Anything else is not the mesh this draws."""
    side = int(round(math.sqrt(vertices)))
    if side * side != vertices:
        raise ValueError(f"{vertices} vertices are not a square grid")
    return side


class TerrainViz(GraphMember):

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting terrain params...")

        mesh = param(self, "terrain_mesh_file", "")
        texture = param(self, "terrain_texture_file", "")
        surface = param(self, "terrain_surface_file", "")
        self.mesh_path = Path(mesh) if mesh else None
        self.texture_path = Path(texture) if texture else None
        self.surface_path = Path(surface) if surface else None
        # One vertex in every stride, each way. The message carries six values
        # for every triangle corner, so a stride of two costs a quarter of it.
        self.stride = max(1, int(param(self, "terrain_stride", 1)))
        # The satellite image, at most this wide. The scene is 600 m across, so
        # 2048 is under a third of a metre a pixel and the model stays small
        # enough to sit in one message.
        self.texture_px = int(param(self, "terrain_texture_px", 2048))
        self.alpha = float(param(self, "terrain_alpha", 1.0))
        self.geoid_height = float(param(self, "geoid_height_m", 0.0))
        self.REFERENCE_FRAME = param(self, "reference_frame", "map")
        local_fix_topic = param(self, "local_fix_topic", "home_position/fix")
        viz_topic = param(self, "terrain_viz_topic", "/viz/scene/terrain")

        self.scene_pub = self.create_publisher(SceneUpdate, viz_topic, LATCHED_QOS)
        # The model as it was last built, and the mesh file it was built from.
        # The scene moves far more often than it is rebuilt, and a move needs
        # the bytes rather than the COLLADA parse and the image behind them.
        self.model_bytes = None
        self.model_note = ""
        self.scene_origin_lla = None
        # The mtime the held model was built from. No file has been read yet,
        # and an mtime never takes this value.
        self.model_stamp = 0

        if self.mesh_path is None or self.surface_path is None:
            self.ground = None
            self.get_logger().warn(
                "no terrain mesh, so the 3D panel shows no ground. A scene "
                "names one; an aircraft outside a built scene does not.")
            self.scene_pub.publish(SceneUpdate(deletions=[], entities=[]))
        else:
            self.ground = SceneGround(self, self.REFERENCE_FRAME, local_fix_topic,
                                      self.geoid_height)
            self.create_timer(RELOAD_CHECK_S, self.publish_when_changed)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Terrain visualization initialized!")

    def publish_when_changed(self) -> None:
        """Draw the scene again when its mesh changes, and when the ground it
        stands on moves under it. The mesh is read once for each change and
        the ground is looked at every tick, because the ground moves far more
        often than the scene is rebuilt."""
        try:
            stamp = self.mesh_path.stat().st_mtime_ns
        except OSError:
            stamp = None
        if stamp != self.model_stamp:
            self.model_stamp = stamp
            self.ground.drawn_at = None
            if stamp is None:
                self.get_logger().info(
                    f"no terrain mesh at {self.mesh_path}. The 3D panel shows "
                    f"no ground until scenegen build writes it.")
            if not self.build_model():
                self.scene_pub.publish(SceneUpdate(deletions=[], entities=[]))
                return
        if self.model_bytes is None:
            return
        offset = self.ground.offset(self.scene_origin_lla)
        if not self.ground.moved(offset):
            return
        self.ground.drawn_at = offset
        self.scene_pub.publish(self.placed(offset))

    def mesh(self):
        """The terrain as a grid of positions and texture coordinates."""
        root = ElementTree.parse(self.mesh_path).getroot()
        positions = float_array(root, "positions").reshape(-1, 3)
        uvs = float_array(root, "uvs").reshape(-1, 2)
        side = grid_side(len(positions))
        return (positions.reshape(side, side, 3)[::self.stride, ::self.stride],
                uvs.reshape(side, side, 2)[::self.stride, ::self.stride])

    def texture(self):
        """The satellite image, no wider than asked for. A model carries its
        image, so a scene's full raster would go out on every publish."""
        image = Image.open(self.texture_path)
        if max(image.size) > self.texture_px:
            image.thumbnail((self.texture_px, self.texture_px))
        packed = io.BytesIO()
        image.convert("RGB").save(packed, "JPEG", quality=88)
        return packed.getvalue(), image.size

    def build_model(self) -> bool:
        """Read the mesh and its image and hold them as one textured model, in
        scene centre coordinates. False where the mesh cannot be drawn, which
        clears the panel.

        Where it stands rides on the primitive's pose, so a ground that moves
        under the scene needs no new model.
        """
        self.model_bytes = None
        self.scene_origin_lla = self.scene_origin()
        if self.scene_origin_lla is None or not self.mesh_path.is_file():
            return False
        try:
            positions, uvs = self.mesh()
            rows, columns = positions.shape[:2]
            corners = np.arange(rows * columns).reshape(rows, columns)
            # Two triangles for each cell, wound counter-clockwise from above.
            upper_left = corners[:-1, :-1]
            upper_right = corners[:-1, 1:]
            lower_left = corners[1:, :-1]
            lower_right = corners[1:, 1:]
            indices = np.stack([upper_left, upper_right, lower_right,
                                upper_left, lower_right, lower_left],
                               axis=-1).ravel()
            image, size = self.texture()
            self.model_bytes = gltf.textured_mesh(
                positions.reshape(-1, 3), uvs.reshape(-1, 2), indices, image,
                "image/jpeg")
        except (OSError, ValueError, ElementTree.ParseError) as error:
            self.model_bytes = None
            self.get_logger().error(f"cannot draw {self.mesh_path}: {error}")
            return False
        self.model_note = (f"{len(indices) // 3} triangles and a "
                           f"{size[0]}x{size[1]} image from "
                           f"{self.mesh_path.name}, "
                           f"{len(self.model_bytes) // 1024} kB")
        return True

    def scene_origin(self):
        """Where the scene centre sits on the Earth, as its surface file holds
        it. None when the file cannot say, which draws nothing."""
        try:
            scene = json.loads(self.surface_path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().error(f"cannot read {self.surface_path}: {error}")
            return None
        origin = scene.get("origin_lla")
        if not origin or len(origin) < 3:
            self.get_logger().error(
                f"{self.surface_path} carries no origin_lla, so the terrain "
                f"cannot be placed against {self.REFERENCE_FRAME}. Build the "
                f"scene again.")
            return None
        return origin

    def placed(self, offset) -> SceneUpdate:
        """The held model standing at `offset` in the reference frame."""
        east, north, up = (float(value) for value in offset)
        primitive = ModelPrimitive(media_type="model/gltf-binary",
                                   data=list(self.model_bytes))
        primitive.pose.position.x = east
        primitive.pose.position.y = north
        primitive.pose.position.z = up
        primitive.pose.orientation = Quaternion(w=1.0)
        primitive.scale = Vector3(x=1.0, y=1.0, z=1.0)
        primitive.color = Color(r=1.0, g=1.0, b=1.0, a=self.alpha)

        entity = SceneEntity(id=ENTITY_ID, frame_id=self.REFERENCE_FRAME,
                             frame_locked=True, models=[primitive])
        entity.timestamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"{self.model_note}, in frame {self.REFERENCE_FRAME}, scene centre "
            f"at ({east:+.3f}, {north:+.3f}, {up:+.3f}) m")
        return SceneUpdate(deletions=[], entities=[entity])
