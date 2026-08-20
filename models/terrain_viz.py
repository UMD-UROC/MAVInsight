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
buildings_viz reconciles. The surface file carries `origin_lla`, which IS the
scene centre, so this node measures it against the home fix and moves every
vertex by that much.

Nothing goes out before the first home fix. Terrain at the wrong offset looks
correct and is wrong, which is worse than an empty panel.

Subscribes
    <local_fix_topic>       sensor_msgs/NavSatFix, the WGS84 position of
                            <reference_frame>
Publishes
    <terrain_viz_topic>     visualization_msgs/MarkerArray, latched
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
from sensor_msgs.msg import NavSatFix

# MAVInsight imports
from models import gltf
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember

COLLADA = "{http://www.collada.org/2005/11/COLLADASchema}"
ENTITY_ID = "scene_terrain"
# How far the home fix moves before the scene is placed again.
HOME_MOVE_M = 1.0
RELOAD_CHECK_S = 5.0

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
reliable_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
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


def as_fix(lla) -> NavSatFix:
    fix = NavSatFix()
    fix.latitude, fix.longitude, fix.altitude = (float(v) for v in lla[:3])
    return fix


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
        self.local_fix = None
        self.published_stamp = 0

        if self.mesh_path is None or self.surface_path is None:
            self.get_logger().warn(
                "no terrain mesh, so the 3D panel shows no ground. A scene "
                "names one; an aircraft outside a built scene does not.")
            self.scene_pub.publish(SceneUpdate(deletions=[], entities=[]))
        else:
            self.create_subscription(NavSatFix, local_fix_topic,
                                     self.local_fix_cb, reliable_qos)
            self.create_timer(RELOAD_CHECK_S, self.publish_when_changed)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Terrain visualization initialized!")

    def local_fix_cb(self, msg: NavSatFix) -> None:
        """The scene is placed against this fix, so a home that moves takes
        the terrain with it."""
        if self.local_fix is not None and self.moved(msg) < HOME_MOVE_M:
            return
        self.local_fix = msg
        self.published_stamp = 0
        self.publish_when_changed()

    def moved(self, fix: NavSatFix) -> float:
        east, north, up = lla_2_enu(self.local_fix, fix, ignore_alt=False)
        return math.sqrt(east ** 2 + north ** 2 + up ** 2)

    def publish_when_changed(self) -> None:
        if self.local_fix is None:
            self.get_logger().warn(
                f"no fix on {self.REFERENCE_FRAME} yet, so the terrain waits. "
                f"Placed against the wrong home it would lie metres off the "
                f"ground it belongs to.", throttle_duration_sec=10.0)
            return
        try:
            stamp = self.mesh_path.stat().st_mtime_ns
        except OSError:
            stamp = None
        if stamp == self.published_stamp:
            return
        self.published_stamp = stamp
        if stamp is None:
            self.get_logger().info(
                f"no terrain mesh at {self.mesh_path}. The 3D panel shows no "
                f"ground until scenegen build writes it.")
        self.scene_pub.publish(self.terrain())

    def scene_offset(self):
        """Where the scene centre sits in the reference frame. The same
        translation buildings_viz applies, so the ground and the buildings on
        it stand in one place."""
        scene = json.loads(self.surface_path.read_text())
        origin = scene.get("origin_lla")
        if not origin or len(origin) < 3:
            raise ValueError(f"{self.surface_path} carries no origin_lla")
        return lla_2_enu(self.local_fix, self.scene_origin(origin), ignore_alt=False)

    def scene_origin(self, origin) -> NavSatFix:
        """The scene centre as a fix in the datum the vehicle reports. A scene
        is anchored above mean sea level, because elevation data is, and a
        NavSatFix is above the ellipsoid. The two are a geoid separation apart,
        33 m in Maryland, which is the whole height of what is drawn."""
        latitude, longitude, mean_sea_level = (float(v) for v in origin[:3])
        return as_fix((latitude, longitude, mean_sea_level + self.geoid_height))

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

    def model(self, positions, uvs):
        """The terrain as one textured model, in scene centre coordinates. The
        offset goes on the primitive's pose, so a home fix that moves needs no
        new model."""
        rows, columns = positions.shape[:2]
        corners = np.arange(rows * columns).reshape(rows, columns)
        # Two triangles for each cell, wound counter-clockwise from above.
        upper_left = corners[:-1, :-1]
        upper_right = corners[:-1, 1:]
        lower_left = corners[1:, :-1]
        lower_right = corners[1:, 1:]
        indices = np.stack([upper_left, upper_right, lower_right,
                            upper_left, lower_right, lower_left], axis=-1).ravel()
        image, size = self.texture()
        model = gltf.textured_mesh(positions.reshape(-1, 3), uvs.reshape(-1, 2),
                                   indices, image, "image/jpeg")
        return model, len(indices) // 3, size

    def terrain(self) -> SceneUpdate:
        if not self.mesh_path.is_file():
            return SceneUpdate(deletions=[], entities=[])
        try:
            offset = self.scene_offset()
            positions, uvs = self.mesh()
            model, triangles, size = self.model(positions, uvs)
        except (OSError, ValueError, ElementTree.ParseError) as error:
            self.get_logger().error(f"cannot draw {self.mesh_path}: {error}")
            return SceneUpdate(deletions=[], entities=[])

        east, north, up = offset
        primitive = ModelPrimitive(media_type="model/gltf-binary", data=list(model))
        primitive.pose.position.x = float(east)
        primitive.pose.position.y = float(north)
        primitive.pose.position.z = float(up)
        primitive.pose.orientation = Quaternion(w=1.0)
        primitive.scale = Vector3(x=1.0, y=1.0, z=1.0)
        primitive.color = Color(r=1.0, g=1.0, b=1.0, a=self.alpha)

        entity = SceneEntity(id=ENTITY_ID, frame_id=self.REFERENCE_FRAME,
                             frame_locked=True, models=[primitive])
        entity.timestamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"{triangles} triangles and a {size[0]}x{size[1]} image from "
            f"{self.mesh_path.name}, {len(model) // 1024} kB, in frame "
            f"{self.REFERENCE_FRAME}, scene centre at "
            f"({east:+.1f}, {north:+.1f}, {up:+.1f}) m")
        return SceneUpdate(deletions=[], entities=[entity])
