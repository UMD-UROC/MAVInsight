"""Draw the scene's building surfaces in the 3D panel.

`scenegen build` writes `worlds/<scene>_buildings.json` next to the world, and
`umd_uas/terrain.py` reads the same buildings to meet a detection ray with a
roof. Nothing drew them, so the panel held terrain and no structures. This
node only draws them.

Horizontal surfaces only, and no walls. A level is one floor of one building,
and a floor is what a camera above looks down onto, which is the same set of
surfaces the localization surface holds. Walls hide the floors behind them
from an operator who looks across the scene, and a ray that meets a wall lands
on the terrain behind it anyway. `extruded` therefore changes nothing here: a
building of floating floors draws its levels exactly as a building that stands
on the ground does.

A timer watches the file's mtime, so a scene rebuilt while the stack runs
appears within a few seconds and needs no restart. One entity carries the
whole scene, so a building that left it also leaves the display.

The file measures every coordinate from the scene centre, and the model goes
in a frame centred on the vehicle's home position. Those are two different
points: the simulator parks each vehicle a few metres from the one before it,
so only the first vehicle takes off at the scene centre. The file carries
`origin_lla`, which IS the scene centre, so this node measures it against the
home fix and puts that on the model's pose. The launch then gives it the
vehicle's own home frame and no launch has to know where a vehicle stands. A
roof drawn any other way disagrees with the terrain the same building is
localized against.

Nothing goes out before the first home fix. Buildings at the wrong offset look
correct and are wrong, which is worse than an empty panel.

The scene's satellite image goes over the roofs, the same image and the same
mapping the terrain is drawn with, so the map reads as one picture draped over
the ground and the buildings on it rather than stopping at every wall. The
terrain's own texture coordinates are a straight map over the scene square to
within half a pixel, so a roof takes the piece of the map it stands on and
lines up with the ground around it.

That is why the buildings go out as a model rather than as coloured triangles:
Foxglove draws no texture on a triangle list, and a roof is one polygon whose
corners are its only vertices, so a colour for each corner would wash the
whole building into one shade.

Subscribes
    <local_fix_topic>       sensor_msgs/NavSatFix, the WGS84 position of
                            <reference_frame>
Publishes
    <buildings_viz_topic>   foxglove_msgs/SceneUpdate, latched
"""

# python imports
import io
import json
import math
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
from models.qos_profiles import reliable_qos


# --------------------------------------------------------------- appearance
LEVEL_FORMAT = "scenegen-buildings/2"
# Format 1 holds one roof for each building, triangulated already. It is read
# as a building of one level, so a scene built before the format changed still
# draws.
ROOF_FORMAT = "scenegen-buildings/1"
ENTITY_ID = "scene_buildings"
# How wide the square a scene covers, where its surface file does not say.
# Only the image over the roofs depends on it.
DEFAULT_SCENE_SIDE_M = 600.0
DEFAULT_SURFACE_COLOR = (0.72, 0.72, 0.74)
SURFACE_ALPHA = 1.0
# How often to look for a changed file. One stat call costs nothing, and a
# rebuilt scene reaches the panel within this long.
RELOAD_CHECK_S = 2.0
# How far the home fix has to move before the scene is drawn again. MAVROS
# republishes the home position on its own, and jitter under this is not a
# move that any building can be seen to make.
HOME_MOVE_M = 1.0

# The panel needs every building from the moment it connects, and the scene
# changes only when someone rebuilds it, so the last message is the whole
# state.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def param(node, name, default):
    """A parameter's value, or its default with the base class's warning."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    node.default_parameter_warning(name)
    return default


# ------------------------------------------------------------ triangulation
def signed_area(ring):
    """Twice the area of a ring. Positive when it winds counter-clockwise."""
    return sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(ring, ring[1:] + ring[:1]))


def wound(ring, counter_clockwise):
    """The ring in the asked winding, as (x, y) pairs.

    A ring is open in the file. A writer that closes one anyway would leave a
    zero length edge, which no ear fits, so the repeated point goes.
    """
    ring = [(float(point[0]), float(point[1])) for point in ring]
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring = ring[:-1]
    if (signed_area(ring) > 0) == counter_clockwise:
        return ring
    return ring[::-1]


def turn(a, b, c):
    """The z of (b - a) x (c - a). Positive is a left turn."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def in_triangle(a, b, c, point):
    """Is the point in the counter-clockwise triangle, its edges included?"""
    return (turn(a, b, point) >= 0 and turn(b, c, point) >= 0
            and turn(c, a, point) >= 0)


def counter_clockwise(a, b, c):
    """The three points wound counter-clockwise, for `in_triangle`."""
    return (a, b, c) if turn(a, b, c) >= 0 else (a, c, b)


def is_reflex(before, point, after):
    return turn(before, point, after) <= 0


def seam_index(ring, hole_point):
    """The vertex of `ring` that a seam from `hole_point` can be cut to.

    Cast a ray from the hole's leftmost vertex toward -x. The ring winds
    counter-clockwise, so its inside is left of every edge and only an edge
    that descends can face the ray. Take the crossing nearest the hole, and of
    that edge the endpoint nearest the hole in x. A reflex vertex inside the
    triangle of hole point, crossing and endpoint would sit across that seam,
    so the one at the smallest angle from the ray replaces it.
    """
    hole_x, hole_y = hole_point
    crossing_x = -math.inf
    index = None
    for position, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
        if a[1] == b[1] or not b[1] <= hole_y <= a[1]:
            continue
        edge_x = a[0] + (hole_y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        if crossing_x < edge_x <= hole_x:
            crossing_x = edge_x
            index = position if a[0] > b[0] else (position + 1) % len(ring)
    if index is None:
        return None

    corner = ring[index]
    blocker = counter_clockwise(hole_point, (crossing_x, hole_y), corner)
    closest = math.inf
    for position, point in enumerate(ring):
        if not corner[0] <= point[0] < hole_x or point == corner:
            continue
        if not in_triangle(*blocker, point):
            continue
        if not is_reflex(ring[position - 1], point,
                         ring[(position + 1) % len(ring)]):
            continue
        angle = abs(hole_y - point[1]) / (hole_x - point[0])
        if angle < closest:
            closest = angle
            index = position
    return index


def cut_holes(footprint, holes):
    """One ring that walks the footprint and every hole, joined by seams.

    A seam is a pair of coincident edges, so the ring stays a simple polygon
    and each hole keeps the opposite winding: an ear cannot cross a seam, and
    the courtyard stays empty. Holes go in from the left, so a seam is cut
    against a ring that no later hole has reached yet.

    A hole that no seam reaches is left out. A filled courtyard is a smaller
    error than a building with no roof.
    """
    ring = wound(footprint, counter_clockwise=True)
    inner = sorted((wound(hole, counter_clockwise=False)
                    for hole in holes if len(hole) >= 3),
                   key=lambda hole: min(point[0] for point in hole))
    for hole in inner:
        start = min(range(len(hole)), key=lambda index: hole[index])
        index = seam_index(ring, hole[start])
        if index is None:
            continue
        ring = (ring[:index + 1] + hole[start:] + hole[:start + 1]
                + ring[index:])
    return ring


def is_ear(ring, remaining, position):
    """Can the vertex at this position be clipped off as a triangle?"""
    count = len(remaining)
    a = ring[remaining[position - 1]]
    b = ring[remaining[position]]
    c = ring[remaining[(position + 1) % count]]
    if turn(a, b, c) <= 0:
        return False
    for other in range(count):
        if other in ((position - 1) % count, position, (position + 1) % count):
            continue
        point = ring[remaining[other]]
        # Only a reflex vertex can hold a triangle open, and only that test
        # keeps the doubled vertices of a seam from blocking every ear.
        if (in_triangle(a, b, c, point)
                and is_reflex(ring[remaining[other - 1]], point,
                              ring[remaining[(other + 1) % count]])):
            return False
    return True


def ear_clip(ring):
    """The triangles of one simple polygon, counter-clockwise.

    O(n^2), and a building footprint holds tens of vertices.

    A ring that touches or crosses itself runs out of ears with vertices
    left, and this raises rather than clipping one anyway. A merged
    footprint that doubles back on itself takes a wrong triangle across the
    whole building, which reads as a roof over the street beside it. A
    building the caller leaves out reads as what it is.
    """
    remaining = list(range(len(ring)))
    triangles = []
    position = 0
    skipped = 0
    while len(remaining) > 3:
        position %= len(remaining)
        if not is_ear(ring, remaining, position):
            position += 1
            skipped += 1
            if skipped > len(remaining):
                raise ValueError(
                    "no ear fits the footprint, so it is not a simple "
                    "polygon: it crosses or touches itself")
            continue
        triangles.append((ring[remaining[position - 1]],
                          ring[remaining[position]],
                          ring[remaining[(position + 1) % len(remaining)]]))
        remaining.pop(position)
        skipped = 0
    if len(remaining) == 3:
        triangles.append(tuple(ring[index] for index in remaining))
    return triangles


def triangulate(footprint, holes):
    """Triangles that fill a footprint and leave its courtyards empty.

    A footprint is concave as often as not, so a fan from one vertex would
    cover ground the building does not stand on. Raises ValueError on a
    footprint that is not a simple polygon.
    """
    if len(footprint) < 3:
        return []
    return ear_clip(cut_holes(footprint, holes))


# ---------------------------------------------------------------- surfaces
def flat_image(color=DEFAULT_SURFACE_COLOR) -> bytes:
    """One color, as an image. A model carries a texture and nothing else, so
    a scene with no satellite raster still draws its roofs in a plain shade
    rather than not at all."""
    packed = io.BytesIO()
    Image.new("RGB", (2, 2), tuple(int(255 * channel) for channel in color)) \
        .save(packed, "JPEG", quality=90)
    return packed.getvalue()


def scene_side(path) -> float:
    """How wide the scene's square is, out of the surface file that decides
    it. The image over the roofs is mapped across that square, so this is the
    same number the terrain is drawn with."""
    try:
        return float(json.loads(Path(path).read_text())["side_m"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return DEFAULT_SCENE_SIDE_M


def satellite_uvs(points, side_m: float):
    """Where each roof corner falls on the scene's satellite image.

    The terrain's own texture coordinates are a straight map over the scene
    square to within half a pixel, so this is that map: a roof takes the piece
    of the image it stands on and registers with the ground around it. v is
    measured up from the bottom of the image, which is what gltf.py expects.
    """
    scene = np.asarray(points, dtype=float)
    return np.column_stack([(scene[:, 0] + side_m / 2.0) / side_m,
                            (scene[:, 1] + side_m / 2.0) / side_m])


def level_surfaces(building):
    """Every level of one building, as triangle corners in scene coordinates.

    The levels of a building share its footprint, so the triangulation is
    done once and lifted to each level's own height. Raises ValueError on a
    footprint that is not a simple polygon.
    """
    mesh = triangulate(building.get("footprint") or [],
                       building.get("holes") or [])
    points = []
    for level in building.get("levels") or []:
        height = float(level.get("z", 0.0))
        for triangle in mesh:
            points += [(corner[0], corner[1], height) for corner in triangle]
    return points


def roof_surfaces(building):
    """The one roof of a `scenegen-buildings/1` building, walls dropped.

    That format carries the roof as triangles already, so it needs no
    triangulation. Its own per corner colors are not read: the satellite image
    goes over every roof, whichever format wrote it.
    """
    roof = building.get("roof") or {}
    return [(float(x), float(y), float(z)) for x, y, z in roof.get("points") or []]


def as_fix(lla) -> NavSatFix:
    """Three numbers from the file as a fix, for the ENU conversion."""
    return NavSatFix(latitude=float(lla[0]), longitude=float(lla[1]),
                     altitude=float(lla[2]))


class BuildingsViz(GraphMember):

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting buildings params...")

        configured = param(self, "buildings_file", "")
        self.path = Path(configured) if configured else None
        self.REFERENCE_FRAME = param(self, "reference_frame", "map")
        self.geoid_height = float(param(self, "geoid_height_m", 0.0))
        # The scene's own image and how wide the square it covers is, so a
        # roof takes the piece of the map it stands on.
        texture = param(self, "terrain_texture_file", "")
        self.texture_path = Path(texture) if texture else None
        self.texture_px = int(param(self, "terrain_texture_px", 2048))
        self.side_m = scene_side(param(self, "terrain_surface_file", ""))
        local_fix_topic = param(self, "local_fix_topic", "home_position/fix")
        viz_topic = param(self, "buildings_viz_topic", "/viz/scene/buildings")

        self.scene_pub = self.create_publisher(SceneUpdate, viz_topic, LATCHED_QOS)
        # Where the frame the model goes in sits on the Earth.
        self.local_fix = None
        # No file has been read yet. An mtime never takes this value.
        self.published_stamp = 0

        if self.path is None:
            self.get_logger().warn(
                "no buildings_file, so the 3D panel shows no structures. A "
                "scene names one; an aircraft outside a built scene does not.")
            self.scene_pub.publish(self.empty())
        else:
            self.create_subscription(NavSatFix, local_fix_topic,
                                     self.local_fix_cb, reliable_qos)
            self.create_timer(RELOAD_CHECK_S, self.publish_when_changed)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Buildings visualization initialized!")

    def local_fix_cb(self, msg: NavSatFix) -> None:
        """The scene is placed against this fix, so a home that moves takes
        the buildings with it."""
        if self.local_fix is not None and self.moved(msg) < HOME_MOVE_M:
            return
        self.local_fix = msg
        self.published_stamp = 0
        self.publish_when_changed()

    def moved(self, fix: NavSatFix) -> float:
        """How far a fix is from the one the scene stands against, in metres."""
        east, north, up = lla_2_enu(self.local_fix, fix, ignore_alt=False)
        return math.sqrt(east ** 2 + north ** 2 + up ** 2)

    def publish_when_changed(self) -> None:
        if self.local_fix is None:
            self.get_logger().warn(
                f"no fix on {self.REFERENCE_FRAME} yet, so the buildings wait. "
                f"Placed against the wrong home they would stand metres off "
                f"the ground they belong to.", throttle_duration_sec=10.0)
            return
        try:
            stamp = self.path.stat().st_mtime_ns
        except OSError:
            stamp = None
        if stamp == self.published_stamp:
            return
        self.published_stamp = stamp
        if stamp is None:
            self.get_logger().info(
                f"no buildings file at {self.path}. The 3D panel shows none "
                f"until scenegen build writes it next to the world.")
        self.scene_pub.publish(self.buildings())

    def empty(self) -> SceneUpdate:
        """Nothing to draw, which clears the panel."""
        return SceneUpdate(deletions=[], entities=[])

    def texture(self):
        """The scene's satellite image, no wider than asked for, or a plain
        shade where the scene has none."""
        if self.texture_path is None or not self.texture_path.is_file():
            return flat_image(), (2, 2)
        image = Image.open(self.texture_path)
        if max(image.size) > self.texture_px:
            image.thumbnail((self.texture_px, self.texture_px))
        packed = io.BytesIO()
        image.convert("RGB").save(packed, "JPEG", quality=88)
        return packed.getvalue(), image.size

    def buildings(self) -> SceneUpdate:
        """The buildings on disk, as one textured model. With no usable file
        an empty update goes out, which clears the panel."""
        if not self.path.is_file():
            return self.empty()
        try:
            scene = json.loads(self.path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().error(f"cannot read {self.path}: {error}")
            return self.empty()

        surfaces = {LEVEL_FORMAT: level_surfaces,
                    ROOF_FORMAT: roof_surfaces}.get(scene.get("format"))
        if surfaces is None:
            self.get_logger().error(
                f"{self.path} carries format {scene.get('format')!r}, and this "
                f"node reads {LEVEL_FORMAT!r} or {ROOF_FORMAT!r}. Build the "
                f"scene again.")
            return self.empty()

        origin = scene.get("origin_lla")
        if not origin or len(origin) < 3:
            self.get_logger().error(
                f"{self.path} carries no origin_lla, so the scene cannot be "
                f"placed against {self.REFERENCE_FRAME}. Build the scene "
                f"again.")
            return self.empty()
        # Where the scene centre sits in the reference frame. The same
        # translation umd_uas/terrain.py applies to reach the tile, so a roof
        # here stands on the surface a ray meets there. It rides on the pose,
        # so the model itself is the scene as the file wrote it.
        offset = lla_2_enu(self.local_fix, self.scene_origin(origin), ignore_alt=False)

        points = []
        drawn = 0
        for index, building in enumerate(scene.get("buildings") or []):
            try:
                corners = surfaces(building)
            # A malformed entry costs its own building and no more.
            except (IndexError, TypeError, ValueError) as error:
                self.get_logger().warn(
                    f"{building.get('id') or index} is not drawn: {error}. "
                    f"Build the scene again.")
                continue
            if corners:
                points += corners
                drawn += 1
        if not points:
            return self.empty()

        image, size = self.texture()
        model = gltf.textured_mesh(
            np.asarray(points, dtype=float), satellite_uvs(points, self.side_m),
            np.arange(len(points), dtype=np.uint32), image, "image/jpeg")

        east, north, up = offset
        primitive = ModelPrimitive(media_type="model/gltf-binary", data=list(model))
        primitive.pose.position.x = float(east)
        primitive.pose.position.y = float(north)
        primitive.pose.position.z = float(up)
        primitive.pose.orientation = Quaternion(w=1.0)
        primitive.scale = Vector3(x=1.0, y=1.0, z=1.0)
        primitive.color = Color(r=1.0, g=1.0, b=1.0, a=SURFACE_ALPHA)

        entity = SceneEntity(id=ENTITY_ID, frame_id=self.REFERENCE_FRAME,
                             frame_locked=True, models=[primitive])
        entity.timestamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"{drawn} buildings, {len(points) // 3} triangles and a "
            f"{size[0]}x{size[1]} image, from {self.path} in frame "
            f"{self.REFERENCE_FRAME}, {len(model) // 1024} kB, scene centre at "
            f"({east:+.1f}, {north:+.1f}, {up:+.1f}) m")
        return SceneUpdate(deletions=[], entities=[entity])

    def scene_origin(self, origin) -> NavSatFix:
        """The scene centre as a fix in the datum the vehicle reports. A scene
        is anchored above mean sea level, because elevation data is, and a
        NavSatFix is above the ellipsoid. The two are a geoid separation apart,
        33 m in Maryland, which is the whole height of what is drawn."""
        latitude, longitude, mean_sea_level = (float(v) for v in origin[:3])
        return as_fix((latitude, longitude, mean_sea_level + self.geoid_height))
