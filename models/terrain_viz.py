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

The mosaic the vehicle builds is drawn into this same image rather than laid
over it. A raster the vehicle rectified against the same surface this mesh is
cut from describes the same ground, so it belongs in the same picture: one
surface, one depth, and "the map over the terrain" becomes a fact about the
image instead of two sheets a hair apart fighting for the same pixel. The live
view keeps its place on top, because `models/scene_ground.py` already draws
this ground a ground clearance under the one the rays are cast at.

The paste is one affine. The mesh carries a texture coordinate for every
vertex, written from the imagery georeference, and one affine reproduces that
mapping to under a twentieth of a texture pixel over both built scenes, so the
mosaic's own lat/lon box lands on the image where the imagery already puts that
ground.

A site with no built scene has no mesh to draw the map into, and the map is
then the only picture of the ground the operator gets. It goes out on its own
there, as a flat quad on the plane both sides cast at, placed by its own
lat/lon box. Nothing else on the ground reads this topic, so a scene decides
what the map is drawn on and never whether it is drawn.

Subscribes
    <local_fix_topic>       sensor_msgs/NavSatFix, the WGS84 position of
                            <reference_frame>
    <mosaic_overlay_topic>  cdcl_umd_msgs/MosaicOverlay, the vehicle's map so
                            far. Empty disables it.
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
from cdcl_umd_msgs.msg import MosaicOverlay
from foxglove_msgs.msg import Color, ModelPrimitive, SceneEntity, SceneUpdate
from geometry_msgs.msg import Quaternion, Vector3

# MAVInsight imports
from models import gltf
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember
from models.scene_ground import SceneGround, as_fix

COLLADA = "{http://www.collada.org/2005/11/COLLADASchema}"
ENTITY_ID = "scene_terrain"
# How often the mesh file and the ground under it are looked at again. Both
# checks are a stat call and a subtraction, so this is what decides how long an
# operator waits after a home update or a scene rebuild.
RELOAD_CHECK_S = 5.0

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
# The mosaic arrives on whatever durability the link gives it. A subscriber
# that asked for a latched one would match a latched publisher and match
# nothing else at all, which is a silent and total failure; this matches
# either, and misses only a map published before this node came up.
LIVE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
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


def enu_to_uv_affine(positions, uvs):
    """The mesh's own east and north to texture coordinate map, as one affine.

    The mesh carries a texture coordinate per vertex, written from the imagery
    georeference, so this is the map the picture is already registered by
    rather than a stretch across the scene square. Fitted by least squares over
    every vertex; the residual is under a twentieth of a texture pixel on both
    built scenes, because a few hundred metres of web mercator is a plane.

    Answers the 2x3 matrix that takes (1, east, north) to (u, v).
    """
    design = np.column_stack([np.ones(len(positions)), positions[:, 0],
                              positions[:, 1]])
    return np.linalg.lstsq(design, uvs, rcond=None)[0].T


def texture_to_mosaic(affine, texture_size, mosaic_box, mosaic_size):
    """The six numbers PIL warps the mosaic into the texture with.

    PIL asks for the map from the output image back to the input, so this is
    texture pixel to scene east and north through the mesh's own affine, then
    east and north to mosaic pixel through the box the mosaic was exported
    with. Both are affine, so the pair is one affine.

    `mosaic_box` is (west, south, east, north) in scene metres, at the centres
    of the mosaic's corner pixels, which is what its bounds name.
    """
    texture_w, texture_h = texture_size
    mosaic_w, mosaic_h = mosaic_size
    # Texture pixel to (u, v). v is measured up from the bottom of the image.
    to_uv = np.array([[1.0 / texture_w, 0.0, 0.0],
                      [0.0, -1.0 / texture_h, 1.0],
                      [0.0, 0.0, 1.0]])
    # East and north to (u, v) is the mesh's affine; turn it round.
    to_uv_from_enu = np.array([[affine[0, 1], affine[0, 2], affine[0, 0]],
                               [affine[1, 1], affine[1, 2], affine[1, 0]],
                               [0.0, 0.0, 1.0]])
    to_enu = np.linalg.inv(to_uv_from_enu)
    # East and north to mosaic pixel. The mosaic is north up and west left,
    # so its rows count south.
    west, south, east, north = mosaic_box
    across = (mosaic_w - 1) / (east - west)
    down = (mosaic_h - 1) / (north - south)
    to_pixel = np.array([[across, 0.0, -west * across],
                         [0.0, -down, north * down],
                         [0.0, 0.0, 1.0]])
    return tuple((to_pixel @ to_enu @ to_uv)[:2].ravel())


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
        mosaic_topic = param(self, "mosaic_overlay_topic", "")

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
        # The satellite image at the size it is drawn, and that size. Read
        # once, so a map that changes costs a paste and an encode rather than
        # a second read and resize of the whole raster.
        self.base_image = None
        self.base_px = 0
        # The newest mosaic, and the one the held model was drawn with. A
        # stamp identifies a map here; two maps of the same ground at the same
        # moment are the same map.
        self.mosaic = None
        self.mosaic_drawn = None

        if mosaic_topic:
            self.create_subscription(MosaicOverlay, mosaic_topic,
                                     self.mosaic_cb, LIVE_QOS)
            self.get_logger().info(
                f"[{self.DISPLAY_NAME}]: the vehicle's map is drawn into the "
                f"terrain from {mosaic_topic}")

        if self.mesh_path is None or self.surface_path is None:
            self.get_logger().warn(
                "no terrain mesh, so the 3D panel shows no ground under the "
                "vehicle's map. A scene names one; an aircraft outside a "
                "built scene does not.")
            self.scene_pub.publish(SceneUpdate(deletions=[], entities=[]))
        self.ground = SceneGround(self, self.REFERENCE_FRAME, local_fix_topic,
                                  self.geoid_height)
        self.create_timer(RELOAD_CHECK_S, self.publish_when_changed)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Terrain visualization initialized!")

    def mosaic_cb(self, msg: MosaicOverlay) -> None:
        """Hold the newest map. It is drawn in on the next tick, so a burst of
        captures costs one rebuild rather than one for each."""
        self.mosaic = msg

    def publish_when_changed(self) -> None:
        """Draw the scene again when its mesh changes, when the vehicle's map
        changes, and when the ground it stands on moves under it. The mesh is
        read once for each change and the ground is looked at every tick,
        because the ground moves far more often than the scene is rebuilt."""
        try:
            stamp = None if self.mesh_path is None else self.mesh_path.stat().st_mtime_ns
        except OSError:
            stamp = None
        mosaic_stamp = None if self.mosaic is None else (
            self.mosaic.header.stamp.sec, self.mosaic.header.stamp.nanosec)
        if stamp != self.model_stamp or mosaic_stamp != self.mosaic_drawn:
            self.model_stamp = stamp
            self.mosaic_drawn = mosaic_stamp
            self.ground.drawn_at = None
            if stamp is None and self.mesh_path is not None:
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

    def texture(self, positions, uvs):
        """The image over the terrain, with the vehicle's map drawn into it.

        The satellite raster is read once at the size it is drawn, because a
        model carries its image and a scene's full raster would go out on
        every publish. A mosaic goes over a copy of it, registered by the
        mesh's own texture coordinates, so the map lands on the ground the
        imagery already puts there and the two are one picture.

        Answers the encoded image, its size and what to say about it.
        """
        wanted = self.image_px(positions)
        if self.base_image is None or self.base_px != wanted:
            image = Image.open(self.texture_path)
            if max(image.size) != wanted:
                scale = wanted / max(image.size)
                image = image.resize((max(1, round(image.width * scale)),
                                      max(1, round(image.height * scale))),
                                     Image.BICUBIC)
            self.base_image = image.convert("RGB")
            self.base_px = wanted
        image, note = self.base_image, ""
        if self.mosaic is not None:
            image, note = self.with_mosaic(self.base_image, positions, uvs)
        packed = io.BytesIO()
        image.save(packed, "JPEG", quality=88)
        return packed.getvalue(), image.size, note

    def mosaic_box(self):
        """The mosaic's corners in scene metres, west, south, east and north.

        None where there is no mosaic yet, or where its corners do not make a
        box. The mosaic names a latitude and longitude pair and the scene is
        measured from its own centre, so the two meet here.
        """
        if self.mosaic is None or self.scene_origin_lla is None:
            return None
        anchor = as_fix((self.scene_origin_lla[0], self.scene_origin_lla[1], 0.0))
        west, south, _ = lla_2_enu(
            anchor, as_fix((self.mosaic.sw_lat, self.mosaic.sw_lon, 0.0)))
        east, north, _ = lla_2_enu(
            anchor, as_fix((self.mosaic.ne_lat, self.mosaic.ne_lon, 0.0)))
        if not (east > west and north > south):
            self.get_logger().warn(
                f"the mosaic's corners do not make a box "
                f"({west:.1f}..{east:.1f} east, {south:.1f}..{north:.1f} "
                f"north), so it is not drawn", throttle_duration_sec=30.0)
            return None
        return (west, south, east, north)

    def image_px(self, positions) -> int:
        """How wide to draw the scene's image, in pixels.

        Fine enough to hold the finest thing in it and no finer. With no
        mosaic that is whatever the satellite raster holds. With one it is the
        mosaic's own resolution, because a map drawn coarser than it was built
        throws away the detail the operator asked for. `terrain_texture_px`
        caps both: a model carries its image, so this is what the message
        costs.
        """
        raster = max(Image.open(self.texture_path).size)
        box = self.mosaic_box()
        if box is not None and self.mosaic.width_px > 1:
            per_px = (box[2] - box[0]) / (self.mosaic.width_px - 1)
            side = float(np.max(positions[..., 0]) - np.min(positions[..., 0]))
            if per_px > 0.0:
                raster = max(raster, int(round(side / per_px)))
        return min(self.texture_px, raster)

    def with_mosaic(self, base, positions, uvs):
        """A copy of the satellite image with the vehicle's map over it.

        The map arrives as a PNG whose alpha is its coverage, and as the
        latitude and longitude box it covers. Both corners are put in scene
        coordinates and the map is warped through one affine into the image,
        so nothing is resampled twice and nothing has to know the imagery's
        own georeference.

        Answers the base image unchanged, and says why, if the map cannot be
        read or does not meet the scene.
        """
        box = self.mosaic_box()
        if box is None:
            return base, ""
        try:
            overlay = Image.open(io.BytesIO(bytes(self.mosaic.overlay_png.data)))
        except (OSError, ValueError) as error:
            self.get_logger().warn(f"cannot read the mosaic overlay: {error}",
                                   throttle_duration_sec=30.0)
            return base, ""
        overlay = overlay.convert("RGBA")

        west, south, east, north = box
        coefficients = texture_to_mosaic(
            enu_to_uv_affine(positions.reshape(-1, 3), uvs.reshape(-1, 2)),
            base.size, box, overlay.size)
        placed = overlay.transform(base.size, Image.AFFINE, coefficients,
                                   resample=Image.BILINEAR)
        drawn = base.copy()
        drawn.paste(placed, (0, 0), placed)
        covered = 100.0 * float(np.asarray(placed)[..., 3].astype(bool).mean())
        return drawn, (f", the vehicle's map over {covered:.1f}% of it "
                       f"({overlay.size[0]}x{overlay.size[1]} px, "
                       f"{east - west:.0f}x{north - south:.0f} m)")

    def build_model(self) -> bool:
        """Read the mesh and its image and hold them as one textured model, in
        scene centre coordinates. False where the mesh cannot be drawn, which
        clears the panel.

        Where it stands rides on the primitive's pose, so a ground that moves
        under the scene needs no new model.
        """
        self.model_bytes = None
        if self.mesh_path is None or self.surface_path is None:
            return self.build_mosaic_model()
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
            image, size, mosaic_note = self.texture(positions, uvs)
            self.model_bytes = gltf.textured_mesh(
                positions.reshape(-1, 3), uvs.reshape(-1, 2), indices, image,
                "image/jpeg")
        except (OSError, ValueError, ElementTree.ParseError) as error:
            self.model_bytes = None
            self.get_logger().error(f"cannot draw {self.mesh_path}: {error}")
            return False
        self.model_note = (f"{len(indices) // 3} triangles and a "
                           f"{size[0]}x{size[1]} image from "
                           f"{self.mesh_path.name}{mosaic_note}, "
                           f"{len(self.model_bytes) // 1024} kB")
        return True

    def build_mosaic_model(self) -> bool:
        """Hold the vehicle's map alone, as a flat quad in map centre
        coordinates. False while there is no map yet, or no fix to place one.

        A site with no built scene has no mesh to draw the map into, and the
        map is then the only picture of the ground there is. Both sides meet
        the flat plane there, which is z = 0 of the reference frame, so the
        quad lies on it and the map's own bounds georeference it.
        """
        self.scene_origin_lla = None
        if self.mosaic is None or self.ground.local_fix is None:
            return False
        # The plane sits at the height the fix reports, and `anchor` puts the
        # scene centre back in that datum by adding the geoid height on.
        self.scene_origin_lla = [
            (self.mosaic.sw_lat + self.mosaic.ne_lat) / 2.0,
            (self.mosaic.sw_lon + self.mosaic.ne_lon) / 2.0,
            self.ground.local_fix.altitude - self.geoid_height]
        box = self.mosaic_box()
        if box is None:
            return False
        try:
            image, size = self.mosaic_image()
        except (OSError, ValueError) as error:
            self.get_logger().warn(f"cannot read the mosaic overlay: {error}",
                                   throttle_duration_sec=30.0)
            return False
        west, south, east, north = box
        positions = np.array([[west, south, 0.0], [east, south, 0.0],
                              [east, north, 0.0], [west, north, 0.0]])
        # v is measured up from the bottom of the image, and the map is north
        # up, so the northern corners carry v = 1.
        uvs = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        indices = np.array([0, 1, 2, 0, 2, 3])
        self.model_bytes = gltf.textured_mesh(positions, uvs, indices, image,
                                              "image/jpeg")
        self.model_note = (f"the vehicle's map on the flat plane, a "
                           f"{size[0]}x{size[1]} image over "
                           f"{east - west:.0f}x{north - south:.0f} m, "
                           f"{len(self.model_bytes) // 1024} kB")
        return True

    def mosaic_image(self):
        """The map as an opaque picture, and its size.

        A model's texture is drawn opaque, so the map's coverage alpha goes
        onto black here rather than out as transparency. `terrain_texture_px`
        caps the width the same way it caps a scene's.
        """
        overlay = Image.open(io.BytesIO(bytes(self.mosaic.overlay_png.data)))
        overlay = overlay.convert("RGBA")
        if max(overlay.size) > self.texture_px:
            scale = self.texture_px / max(overlay.size)
            overlay = overlay.resize((max(1, round(overlay.width * scale)),
                                      max(1, round(overlay.height * scale))),
                                     Image.BICUBIC)
        picture = Image.new("RGB", overlay.size)
        picture.paste(overlay, (0, 0), overlay)
        packed = io.BytesIO()
        picture.save(packed, "JPEG", quality=88)
        return packed.getvalue(), picture.size

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
