"""Draw the verdicts that umd_uas scores.

`umd_uas/scoring.py` compares localized detections against ground truth and
publishes two `vision_msgs/Detection3DArray` streams: one entry per estimate
with its verdict, and one entry per ground truth target with its status. This
node only draws them, in every view at once:

    3D panel   a bubble on every target, colored by its status, and a mark on
               every estimate, colored by its verdict
    Image      the detection boxes, each in the color of its own verdict
    Map panel  one layer for each verdict kind, a pin on every estimate, and
               a ring around every known target in the color of its status

The bubble radius comes from `bbox.size` of the message, which carries the gate
the estimate was scored against. Nothing here decides it, so a mark inside a
bubble is a hit by construction.

A target that is in view and that no estimate names is the false negative. It
has no estimate to mark, so it appears as its bubble turning red.

The verdicts draw the image boxes, and the localization does not. A verdict
message names the frame it judged in its header stamp, so this node draws the
boxes of that frame and colors each one with its own verdict: one message
decides both views and they cannot disagree. Verdicts with nothing in them
name no frame, which takes every box off the picture at that instant.

Every view empties itself rather than freezing the last hit. A Map layer with
nothing this tick goes out as an empty collection.

The marker arrays go out on a clock rather than on arrival. A panel redraws
a marker in a fixed frame from the transform tree on every panel frame, so
this rate carries a change of verdict to the operator rather than the
geometry. A tick only restamps arrays the node already holds, so the rate is
nearly free: see the tunables.

Subscribes
    <verdicts_topic>        vision_msgs/Detection3DArray
    <target_status_topic>   vision_msgs/Detection3DArray
    <localization_topic>    cdcl_umd_msgs/TargetBoxArray, for the image boxes
    <local_fix_topic>       sensor_msgs/NavSatFix, the WGS84 position of
                            <localization_frame>, for the Map panel layers
Publishes
    <verdict_viz_topic>      visualization_msgs/MarkerArray
    <target_viz_topic>       visualization_msgs/MarkerArray
    <annotations_topic>      foxglove_msgs/ImageAnnotations
    <true_positive_geojson_topic>, <mislocalized_geojson_topic>,
    <false_positive_geojson_topic>, <target_ring_geojson_topic>
                             foxglove_msgs/GeoJSON, latched
"""

# python imports
import json
import math
from collections import OrderedDict
from typing import NamedTuple

# ROS2 imports
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# ROS2 message imports
from cdcl_umd_msgs.msg import TargetBoxArray
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

# MAVInsight imports
from models.frame_utils import enu_2_lla
from models.graph_member import GraphMember
from models.qos_profiles import reliable_qos, viz_qos
from models.scene_ground import FrameSurvey

try:
    from foxglove_msgs.msg import (Color, GeoJSON, ImageAnnotations, Point2,
                                   PointsAnnotation, TextAnnotation)
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

# A Map panel layer is the whole state of one verdict kind, so a panel that
# connects late needs the last message and nothing before it.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ----------------------------------------------------------------- tunables
# How often the marks and the bubbles go out. The panels draw what the last
# message carried, so this is the rate an operator sees a verdict change at.
# A tick restamps arrays that are already built, which measures 0.05 ms
# against the 4.6 ms it costs to build them, so the rate is nearly free.
MARKER_RATE_HZ = 10.0
# How long the boxes stay on the picture after the scorer stops publishing.
# The verdicts take the boxes off, so this covers only a scorer that has gone
# quiet: a backstop rather than the mechanism. A backstop that fires in normal
# running is not a backstop, so it is several scoring periods.
SCORER_SILENCE_S = 2.0
# How many localization frames this node holds while the scorer judges them.
# The scorer judges a frame as it arrives, so one or two are ever in flight,
# and tf_loc localizes frames concurrently, so they can arrive out of order.
FRAMES_HELD = 8


# --------------------------------------------------------------- appearance
# Edit this block and the 3D marks, the bubbles and the image boxes all follow.
#   TP            the estimate lies within the gate of a target
#   MISLOCALIZED  the viewing ray crosses the gate of a target, but the
#                 estimate lies within the gate of none
#   FP            the ray crosses the gate of no target
VERDICT_COLOR = {
    "TP": (0.18, 0.80, 0.44, 0.95),
    "MISLOCALIZED": (0.95, 0.77, 0.06, 0.95),
    "FP": (0.91, 0.30, 0.24, 0.95),
}
# The same three verdicts on the Map panel, as CSS colors. True red, yellow
# and green rather than the muted tones above: a flat map pin carries no
# shading to soften, and the operator reads it at a glance.
MAP_VERDICT_COLOR = {
    "TP": "#00ff00",
    "MISLOCALIZED": "#ffff00",
    "FP": "#ff0000",
}
# What a verdict cites, as umd_uas/scoring.py writes it. Each citation carries
# the ground distance to that target.
HIT = "hit:"
CROSSED = "crossed:"
NEAREST = "nearest:"

# Which Map panel layer each verdict goes to, and the parameter that renames
# it. An FN has no estimate to place, so it has no entry: it shows as its
# target bubble turning red.
MAP_VERDICT_TOPIC = {
    "TP": ("true_positive_geojson_topic", "/viz/scoring/true_positives"),
    "MISLOCALIZED": ("mislocalized_geojson_topic", "/viz/scoring/missed_localizations"),
    "FP": ("false_positive_geojson_topic", "/viz/scoring/false_positives"),
}
MAP_RING_TOPIC = ("target_ring_geojson_topic", "/viz/scoring/target_rings")

# The same four target statuses on the Map panel, as CSS colors. A target
# nobody is looking at is grey rather than red: it has not been missed, it has
# not been seen.
MAP_STATUS_COLOR = {
    "detected": "#00ff00",
    "mislocalized": "#ffff00",
    "visible": "#ff0000",
    "out_of_view": "#9aa0a6",
}
# A ring this far around each known target, drawn as a line rather than a
# filled shape so the imagery under it stays readable. A scene holds hundreds
# of targets and every ring goes out whole, so the corner count is the size of
# the layer: twelve of them stand less than a tenth of a metre inside a circle
# this small, which is nothing an operator can see.
TARGET_RING_RADIUS_M = 2.0
TARGET_RING_POINTS = 12
TARGET_RING_WEIGHT = 2

# Verdicts drawn as a flat cross rather than a dot: they mark a problem.
CROSS_VERDICTS = frozenset({"MISLOCALIZED", "FP"})
# An image box that no verdict has judged yet.
UNJUDGED_COLOR = (0.75, 0.75, 0.78, 1.0)

# A target's status across the estimates of the last scored frame.
STATUS_COLOR = {
    "detected": (0.18, 0.80, 0.44, 0.30),
    "mislocalized": (0.95, 0.77, 0.06, 0.30),
    "visible": (0.91, 0.30, 0.24, 0.30),
    "out_of_view": (0.55, 0.55, 0.58, 0.25),
}

# An estimate is a small dot on the ground, not a person-sized shape.
DETECTION_DOT_DIAMETER = 0.4
DETECTION_CROSS_SPAN = 1.2
DETECTION_CROSS_LINE_WIDTH = 0.15
# Marks float this far above the ground, clear of z fighting.
MARK_LIFT_M = 0.05
LABEL_HEIGHT_M = 0.7
LABEL_COLOR = (0.92, 0.94, 0.96, 0.9)

BOX_LINE_THICKNESS = 2.0
BOX_LABEL_FONT_SIZE = 14.0
BOX_LABEL_TEXT_COLOR = (1.0, 1.0, 1.0, 1.0)
BOX_LABEL_BACKGROUND = (0.0, 0.0, 0.0, 0.6)


class Box(NamedTuple):
    """One detection box in the picture it was measured on, ready to draw.

    This node works the corners and the label out as the frame arrives, so a
    verdict that colors them has nothing left to compute.
    """
    verdict_id: str
    label: str
    left: float
    top: float
    right: float
    bottom: float


def frame_key(stamp) -> tuple:
    """What names one frame. A message stamp is two integers, and a message is
    not hashable, so this is the key a frame is held under."""
    return (stamp.sec, stamp.nanosec)


def param(node, name, default):
    """A parameter's value, or its default with the base class's warning."""
    if node.has_parameter(name):
        return node.get_parameter(name).value
    node.default_parameter_warning(name)
    return default


def rgba(color) -> ColorRGBA:
    """A marker color. ROS message constructors take keywords only, so every
    color tuple in this file is spread by name."""
    return ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])


def fox_color(color):
    """The same tuple as a foxglove_msgs Color, for the image annotations."""
    return Color(r=color[0], g=color[1], b=color[2], a=color[3])


def annotation_text(label: str, identifier: str) -> str:
    """What one detection is called, on the image and in the 3D panel alike, so
    an operator can match a mark to the box it came from."""
    return f"{label} {identifier}".strip()


def cited(detection):
    """What one estimate touched, out of the verdict's own citations.

    umd_uas/scoring.py marks each one with what it is and hangs the ground
    distance on it, so this reads them by name rather than by counting what
    came before. Anything unmarked is the detector's own label.
    """
    hit, crossed, nearest, label, confidence = [], [], None, "", 0.0
    for result in detection.results[1:]:
        name, distance = result.hypothesis.class_id, float(result.hypothesis.score)
        if name.startswith(HIT):
            hit.append((name[len(HIT):], distance))
        elif name.startswith(CROSSED):
            crossed.append((name[len(CROSSED):], distance))
        elif name.startswith(NEAREST):
            nearest = (name[len(NEAREST):], distance)
        else:
            label, confidence = name, distance
    return hit, crossed, nearest, label, confidence


def with_distances(targets) -> str:
    return ", ".join(f"{name} {distance:.1f} m" for name, distance in targets)


def map_feature(detection, latitude: float, longitude: float) -> dict:
    """One estimate as a GeoJSON point.

    The Map panel shows `name` and `metadata` in the hover tooltip and reads
    its Leaflet options from `style`. The name is the label the image overlay
    draws, so a pin and its box read as one detection.

    Every verdict carries the same account of itself, so a false positive is
    read the same way a hit is: what it landed in, what its ray went through,
    and what the nearest target was however far off it fell.
    """
    kind = detection.results[0].hypothesis.class_id
    color = MAP_VERDICT_COLOR[kind]
    hit, crossed, nearest, label, confidence = cited(detection)
    metadata = {"verdict": kind}
    if label:
        metadata["confidence"] = round(confidence, 3)
    if hit:
        metadata["in_gate_of"] = with_distances(hit)
    if crossed:
        metadata["ray_crossed"] = with_distances(crossed)
    if nearest:
        metadata["nearest"] = nearest[0]
        metadata["nearest_error_m"] = round(nearest[1], 2)
    return {
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [longitude, latitude]},
        "properties": {
            "name": annotation_text(label, detection.id),
            "metadata": metadata,
            "style": {"color": color, "fillColor": color, "fillOpacity": 0.9},
        },
    }


def ring_feature(name: str, status: str, ring) -> dict:
    """One known target as a ring on the Map panel.

    A LineString rather than a Polygon: a polygon is a filled shape, and the
    panel draws the fill over the imagery the operator is reading the target
    off. `fill` is off as well, so the same holds wherever the ring is drawn
    as an area, and nothing is drawn part transparent.
    """
    color = MAP_STATUS_COLOR.get(status, MAP_STATUS_COLOR["out_of_view"])
    return {
        "type": "Feature",
        "geometry": {"type": "LineString", "coordinates": ring},
        "properties": {
            "name": name,
            "metadata": {"status": status},
            "style": {"color": color, "weight": TARGET_RING_WEIGHT,
                      "opacity": 1.0, "fill": False},
        },
    }


def clear_all(header: Header) -> Marker:
    """Leads every array, so a departed target leaves the display instead of
    standing where it last was."""
    return Marker(header=header, action=Marker.DELETEALL)


# Everything here stands still in a frame that stands still, so these tell a
# panel to place them with the transform tree it draws now rather than with
# the tree that stood when the node wrote the message. That is what
# frame_locked means, and it holds a mark on its target while the aircraft
# maneuvers, whatever rate these go out at.
def sphere(header: Header, ns: str, marker_id: int, position, diameter: float,
           color) -> Marker:
    marker = Marker(
        header=header, ns=ns, id=marker_id, type=Marker.SPHERE,
        action=Marker.ADD, frame_locked=True,
        scale=Vector3(x=diameter, y=diameter, z=diameter),
        color=rgba(color))
    marker.pose.position = position
    marker.pose.orientation.w = 1.0
    return marker


def cross(header: Header, ns: str, marker_id: int, position, span: float,
          color) -> Marker:
    half = span / 2.0
    marker = Marker(
        header=header, ns=ns, id=marker_id, type=Marker.LINE_LIST,
        action=Marker.ADD, frame_locked=True,
        points=[Point(x=-half, y=-half), Point(x=half, y=half),
                Point(x=-half, y=half), Point(x=half, y=-half)],
        scale=Vector3(x=DETECTION_CROSS_LINE_WIDTH),
        color=rgba(color))
    marker.pose.position = position
    marker.pose.orientation.w = 1.0
    return marker


def text(header: Header, ns: str, marker_id: int, position, body: str) -> Marker:
    marker = Marker(
        header=header, ns=ns, id=marker_id, type=Marker.TEXT_VIEW_FACING,
        action=Marker.ADD, frame_locked=True, text=body,
        scale=Vector3(z=LABEL_HEIGHT_M), color=rgba(LABEL_COLOR))
    marker.pose.position = position
    marker.pose.orientation.w = 1.0
    return marker


class ScoringViz(GraphMember):

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting scoring viz params...")

        verdicts_topic = param(self, "verdicts_topic", "scoring/verdicts")
        status_topic = param(self, "target_status_topic", "scoring/target_status")
        localization_topic = param(self, "localization_topic", "target_locations")
        verdict_viz_topic = param(self, "verdict_viz_topic", "/viz/scoring/verdicts")
        target_viz_topic = param(self, "target_viz_topic", "/viz/scoring/targets")
        annotations_topic = param(self, "annotations_topic", "/viz/scoring/annotations")
        local_fix_topic = param(self, "local_fix_topic", "home_position/fix")

        # The boxes of the last few localization frames, by frame stamp,
        # waiting for the verdicts that judge them.
        self.frames = OrderedDict()
        # The marks and the bubbles as they stand. A message that changes
        # them rebuilds them, and a tick between only restamps them.
        self.verdict_markers = None
        self.target_markers = None
        # The bubbles alone, in target order, because their color is the only
        # part of them the status changes.
        self.target_bubbles = []
        # Every target's map ring, and what the rings were built for. Ring
        # corners are the expensive part of a tick and they never move.
        self.ring_features = []
        self.targets_placed = None
        # The last collection published on each Map layer. The layers are
        # latched, so an unchanged one needs no message.
        self.map_drawn = {}
        # The WGS84 position of the frame the verdicts are measured in. The
        # Map panel needs longitude and latitude, and the compute node already
        # publishes this fix for every other view of the same frame.
        self.local_fix = None
        # Frame coordinates go out to the Map panel as WGS84, so the survey
        # goes back on: the same correction umd_uas/footprint.py applies.
        self.survey = FrameSurvey(self, param(self, "localization_frame", "map"))

        self.create_subscription(Detection3DArray, verdicts_topic,
                                 self.verdicts_cb, reliable_qos)
        self.create_subscription(Detection3DArray, status_topic,
                                 self.status_cb, reliable_qos)
        self.verdict_pub = self.create_publisher(MarkerArray, verdict_viz_topic,
                                                 reliable_qos)
        self.target_pub = self.create_publisher(MarkerArray, target_viz_topic,
                                                reliable_qos)

        self.annotation_pub = None
        self.geojson_pub = {}
        self.ring_pub = None
        # Whether boxes are on the picture now, and when the scorer last
        # spoke. The verdicts take the boxes off, so this only catches a
        # scorer that stopped.
        self.boxes_drawn = False
        self.verdict_sec = 0.0
        self.create_timer(1.0 / MARKER_RATE_HZ, self.publish_markers)
        if HAVE_FOXGLOVE:
            self.annotation_pub = self.create_publisher(
                ImageAnnotations, annotations_topic, viz_qos)
            self.create_subscription(TargetBoxArray, localization_topic,
                                     self.localizations_cb, reliable_qos)
            self.create_subscription(NavSatFix, local_fix_topic,
                                     self.local_fix_cb, reliable_qos)
            self.geojson_pub = {
                kind: self.create_publisher(GeoJSON, param(self, name, default),
                                            LATCHED_QOS)
                for kind, (name, default) in MAP_VERDICT_TOPIC.items()
            }
            self.ring_pub = self.create_publisher(
                GeoJSON, param(self, *MAP_RING_TOPIC), LATCHED_QOS)
        else:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Image panel gets no boxes "
                "and the Map panel gets no verdicts. "
                "Install ros-$ROS_DISTRO-foxglove-msgs.")

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Scoring visualization initialized!")

    def verdicts_cb(self, msg: Detection3DArray) -> None:
        """One frame's judgement, in every view that shows a verdict.

        The marks go into the array the clock publishes. The image boxes and
        the Map pins go out here, because each is a statement about this
        message and nothing else says when it changed.
        """
        markers = [clear_all(msg.header)]
        for index, detection in enumerate(msg.detections):
            kind = detection.results[0].hypothesis.class_id if detection.results else "FP"
            color = VERDICT_COLOR.get(kind)
            if color is None:
                continue
            center = detection.bbox.center.position
            position = Point(x=center.x, y=center.y, z=center.z + MARK_LIFT_M)
            build, size = ((cross, DETECTION_CROSS_SPAN) if kind in CROSS_VERDICTS
                           else (sphere, DETECTION_DOT_DIAMETER))
            # Namespaced by verdict, so one kind can be switched off in the 3D
            # panel without touching the others.
            markers.append(build(msg.header, kind, index, position, size, color))
        self.verdict_markers = MarkerArray(markers=markers)
        self.verdict_sec = self.now_sec()
        self.publish_annotations(msg)
        self.publish_map(msg)

    def local_fix_cb(self, msg: NavSatFix) -> None:
        self.local_fix = msg

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def publish_markers(self) -> None:
        """Send the marks and the bubbles again, stamped now.

        A tick restamps and publishes the arrays it already holds. It also
        catches a scorer that stopped: the verdicts take the boxes off the
        picture, and nothing else would.
        """
        stamp = self.get_clock().now().to_msg()
        for markers, publisher in ((self.verdict_markers, self.verdict_pub),
                                   (self.target_markers, self.target_pub)):
            if markers is None:
                continue
            for marker in markers.markers:
                marker.header.stamp = stamp
            publisher.publish(markers)
        if (self.boxes_drawn
                and self.now_sec() - self.verdict_sec > SCORER_SILENCE_S):
            self.get_logger().warn(
                f"no verdict for {SCORER_SILENCE_S:.0f} s: the boxes come off "
                f"the picture", throttle_duration_sec=30.0)
            self.annotation_pub.publish(ImageAnnotations())
            self.boxes_drawn = False

    def publish_annotations(self, verdicts: Detection3DArray) -> None:
        """The boxes of the frame these verdicts judge, each in its own color.

        The verdicts name their frame in the header stamp, so the boxes this
        draws are the boxes that made the marks: one message decides both
        views and they cannot disagree. Verdicts with nothing in them name no
        frame, which takes every box off at that instant.

        The annotations carry the frame's stamp, not the clock's: a stamp
        taken here would slide the boxes off their frame whenever a recording
        is scrubbed.

        A box the scorer could not place carries no verdict, so it goes on
        the picture unjudged: the detector did report it.
        """
        if self.annotation_pub is None:
            return
        held = self.frames.get(frame_key(verdicts.header.stamp))
        if held is None and verdicts.detections:
            # The frame is still on its way. Leaving the picture alone beats
            # blinking the boxes off and straight back on.
            return
        stamp, boxes = held if held is not None else (verdicts.header.stamp, [])
        verdict_for = {detection.id: (detection.results[0].hypothesis.class_id
                                      if detection.results else "FP")
                       for detection in verdicts.detections}

        out = ImageAnnotations()
        for box in boxes:
            color = fox_color(VERDICT_COLOR.get(verdict_for.get(box.verdict_id),
                                                UNJUDGED_COLOR))
            outline = PointsAnnotation()
            outline.timestamp = stamp
            outline.type = PointsAnnotation.LINE_LOOP
            outline.thickness = BOX_LINE_THICKNESS
            outline.outline_color = color
            outline.points = [
                Point2(x=box.left, y=box.top),
                Point2(x=box.right, y=box.top),
                Point2(x=box.right, y=box.bottom),
                Point2(x=box.left, y=box.bottom),
            ]
            out.points.append(outline)

            label = TextAnnotation()
            label.timestamp = stamp
            label.position = Point2(x=box.left,
                                    y=max(box.top - 4.0, BOX_LABEL_FONT_SIZE))
            label.text = box.label
            label.font_size = BOX_LABEL_FONT_SIZE
            label.text_color = fox_color(BOX_LABEL_TEXT_COLOR)
            label.background_color = fox_color(BOX_LABEL_BACKGROUND)
            out.texts.append(label)

        self.annotation_pub.publish(out)
        self.boxes_drawn = bool(out.points)

    def publish_map(self, msg: Detection3DArray) -> None:
        """Every Map panel layer.

        A kind with nothing on it goes out as an empty collection, which takes
        its pins off the panel instead of leaving the last hit there. The
        layers are latched, so an unchanged one needs no message and a panel
        that connects later still gets the last one.
        """
        if not self.geojson_pub:
            return
        if self.local_fix is None:
            self.get_logger().warn(
                "no local fix yet, so the Map panel gets no verdicts",
                throttle_duration_sec=10.0)
            return
        correction = self.survey.correction()
        features = {kind: [] for kind in self.geojson_pub}
        for detection in msg.detections:
            kind = (detection.results[0].hypothesis.class_id
                    if detection.results else None)
            if kind not in features:
                continue
            center = detection.bbox.center.position
            latitude, longitude, _ = enu_2_lla(
                self.local_fix, center.x + correction[0],
                center.y + correction[1], center.z + correction[2])
            features[kind].append(map_feature(detection, latitude, longitude))
        for kind, publisher in self.geojson_pub.items():
            drawn = json.dumps({"type": "FeatureCollection",
                                "features": features[kind]})
            if drawn == self.map_drawn.get(kind):
                continue
            self.map_drawn[kind] = drawn
            publisher.publish(GeoJSON(geojson=drawn))

    def status_cb(self, msg: Detection3DArray) -> None:
        """Every target's bubble and map ring, in the color of its status.

        Targets stand still, so this builds the bubbles, the names and the
        ring corners once and holds them. Building the rings again costs a
        hundred milliseconds and recoloring them costs under one, so only the
        colors follow the status.
        """
        correction = None if self.local_fix is None else self.survey.correction()
        placed = (
            tuple((detection.id,
                   detection.bbox.center.position.x,
                   detection.bbox.center.position.y,
                   detection.bbox.center.position.z,
                   detection.bbox.size.x) for detection in msg.detections),
            None if correction is None else
            (self.local_fix.latitude, self.local_fix.longitude,
             self.local_fix.altitude, tuple(correction)))
        if placed != self.targets_placed:
            self.place_targets(msg, correction)
            self.targets_placed = placed

        for index, detection in enumerate(msg.detections):
            status = (detection.results[0].hypothesis.class_id
                      if detection.results else "out_of_view")
            self.target_bubbles[index].color = rgba(
                STATUS_COLOR.get(status, STATUS_COLOR["out_of_view"]))
            if self.ring_features:
                properties = self.ring_features[index]["properties"]
                properties["metadata"]["status"] = status
                properties["style"]["color"] = MAP_STATUS_COLOR.get(
                    status, MAP_STATUS_COLOR["out_of_view"])
        self.publish_rings()

    def place_targets(self, msg: Detection3DArray, correction) -> None:
        """Build the bubble, the name and the map ring of every target.

        Where the targets are and where their frame sits on the Earth fix
        everything here. Both hold still, so this runs when one of them
        changes rather than on every message.
        """
        markers = [clear_all(msg.header)]
        self.target_bubbles = []
        self.ring_features = []
        for index, detection in enumerate(msg.detections):
            position = detection.bbox.center.position
            # The gate the estimate was scored against, straight off the wire.
            bubble = sphere(msg.header, "targets", index, position,
                            detection.bbox.size.x, STATUS_COLOR["out_of_view"])
            markers.append(bubble)
            self.target_bubbles.append(bubble)
            label_position = Point(x=position.x, y=position.y,
                                   z=position.z + detection.bbox.size.x / 2.0
                                   + LABEL_HEIGHT_M)
            markers.append(text(msg.header, "target_names", index,
                                label_position, detection.id))
            if correction is None:
                continue
            ring = []
            for step in range(TARGET_RING_POINTS + 1):
                angle = 2.0 * math.pi * step / TARGET_RING_POINTS
                latitude, longitude, _ = enu_2_lla(
                    self.local_fix,
                    position.x + TARGET_RING_RADIUS_M * math.cos(angle) + correction[0],
                    position.y + TARGET_RING_RADIUS_M * math.sin(angle) + correction[1],
                    position.z + correction[2])
                ring.append([longitude, latitude])
            self.ring_features.append(
                ring_feature(detection.id, "out_of_view", ring))
        self.target_markers = MarkerArray(markers=markers)

    def publish_rings(self) -> None:
        """The rings, when a status has changed one.

        A scene holds hundreds of targets and every ring goes out whole, so
        this is the same half a megabyte over and over otherwise. The layer is
        latched, so a panel that connects later still gets the last one.
        """
        if self.ring_pub is None or not self.ring_features:
            return
        drawn = json.dumps({"type": "FeatureCollection",
                            "features": self.ring_features})
        if drawn == self.map_drawn.get("rings"):
            return
        self.map_drawn["rings"] = drawn
        self.ring_pub.publish(GeoJSON(geojson=drawn))

    def localizations_cb(self, msg: TargetBoxArray) -> None:
        """Hold one frame's boxes until the verdicts that judge it arrive.

        Nothing is drawn here. A box goes on the picture when its verdict
        does, so the boxes and the marks are always the same detections.
        """
        boxes = []
        for index, box in enumerate(msg.uav_target_boxes):
            half_width = box.target_bbox.size_x / 2.0
            half_height = box.target_bbox.size_y / 2.0
            center = box.target_bbox.center.position
            boxes.append(Box(
                verdict_id=f"{box.data_source_id}_{index}",
                label=annotation_text(box.detection_class or "object",
                                      str(box.data_source_id)),
                left=center.x - half_width, top=center.y - half_height,
                right=center.x + half_width, bottom=center.y + half_height))
        self.frames[frame_key(msg.header.stamp)] = (msg.header.stamp, boxes)
        while len(self.frames) > FRAMES_HELD:
            self.frames.popitem(last=False)
