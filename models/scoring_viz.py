"""Draw the verdicts that umd_uas scores.

`umd_uas/scoring.py` compares localized detections against ground truth and
publishes two `vision_msgs/Detection3DArray` streams: one entry per estimate
with its verdict, and one entry per ground truth target with its status. This
node only draws them, in every view at once:

    3D panel   a bubble on every target, colored by its status, and a mark on
               every estimate, colored by its verdict
    Image      the detection boxes, each in the color of its own verdict
    Map panel  one layer for each verdict kind, a pin on every estimate

The bubble radius comes from `bbox.size` of the message, which carries the gate
the estimate was scored against. Nothing here decides it, so a mark inside a
bubble is a hit by construction.

A target that is in view and that no estimate names is the false negative. It
has no estimate to mark, so it appears as its bubble turning red.

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
    <false_positive_geojson_topic>
                             foxglove_msgs/GeoJSON, latched
"""

# python imports
import json

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
# Which Map panel layer each verdict goes to, and the parameter that renames
# it. An FN has no estimate to place, so it has no entry: it shows as its
# target bubble turning red.
MAP_VERDICT_TOPIC = {
    "TP": ("true_positive_geojson_topic", "/viz/scoring/true_positives"),
    "MISLOCALIZED": ("mislocalized_geojson_topic", "/viz/scoring/missed_localizations"),
    "FP": ("false_positive_geojson_topic", "/viz/scoring/false_positives"),
}

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


def map_feature(detection, latitude: float, longitude: float) -> dict:
    """One estimate as a GeoJSON point.

    The Map panel shows `name` and `metadata` in the hover tooltip and reads
    its Leaflet options from `style`. The name is the label the image overlay
    draws, so a pin and its box read as one detection.
    """
    kind = detection.results[0].hypothesis.class_id
    color = MAP_VERDICT_COLOR[kind]
    detected = detection.results[1].hypothesis if len(detection.results) > 1 else None
    names = [result.hypothesis.class_id for result in detection.results[2:]]
    metadata = {"verdict": kind}
    if detected is not None:
        metadata["confidence"] = round(float(detected.score), 3)
    if names:
        # What this estimate matched, so the pin and the bubble it colored can
        # be read together. The ground error rides in the verdict's own score.
        metadata["targets"] = ", ".join(names)
        metadata["error_m"] = round(float(detection.results[0].hypothesis.score), 2)
    return {
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [longitude, latitude]},
        "properties": {
            "name": annotation_text(detected.class_id if detected else "",
                                    detection.id),
            "metadata": metadata,
            "style": {"color": color, "fillColor": color, "fillOpacity": 0.9},
        },
    }


def clear_all(header: Header) -> Marker:
    """Leads every array, so a departed target leaves the display instead of
    standing where it last was."""
    return Marker(header=header, action=Marker.DELETEALL)


def sphere(header: Header, ns: str, marker_id: int, position, diameter: float,
           color) -> Marker:
    marker = Marker(
        header=header, ns=ns, id=marker_id, type=Marker.SPHERE,
        action=Marker.ADD, scale=Vector3(x=diameter, y=diameter, z=diameter),
        color=rgba(color))
    marker.pose.position = position
    marker.pose.orientation.w = 1.0
    return marker


def cross(header: Header, ns: str, marker_id: int, position, span: float,
          color) -> Marker:
    half = span / 2.0
    marker = Marker(
        header=header, ns=ns, id=marker_id, type=Marker.LINE_LIST,
        action=Marker.ADD,
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
        action=Marker.ADD, text=body, scale=Vector3(z=LABEL_HEIGHT_M),
        color=rgba(LABEL_COLOR))
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

        # The verdict of every estimate of the last scored frame, by its id.
        self.verdict_for = {}
        # The WGS84 position of the frame the verdicts are measured in. The
        # Map panel needs longitude and latitude, and the compute node already
        # publishes this fix for every other view of the same frame.
        self.local_fix = None

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
        else:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Image panel gets no boxes "
                "and the Map panel gets no verdicts. "
                "Install ros-$ROS_DISTRO-foxglove-msgs.")

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Scoring visualization initialized!")

    def verdicts_cb(self, msg: Detection3DArray) -> None:
        self.verdict_for = {
            detection.id: (detection.results[0].hypothesis.class_id
                           if detection.results else "FP")
            for detection in msg.detections
        }

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
        self.verdict_pub.publish(MarkerArray(markers=markers))
        self.publish_map(msg)

    def local_fix_cb(self, msg: NavSatFix) -> None:
        self.local_fix = msg

    def publish_map(self, msg: Detection3DArray) -> None:
        """Every Map panel layer, every tick.

        A kind with nothing this tick publishes an empty collection, which
        takes its pins off the panel instead of leaving the last hit there.
        """
        if not self.geojson_pub:
            return
        if self.local_fix is None:
            self.get_logger().warn(
                "no local fix yet, so the Map panel gets no verdicts",
                throttle_duration_sec=10.0)
            return
        features = {kind: [] for kind in self.geojson_pub}
        for detection in msg.detections:
            kind = (detection.results[0].hypothesis.class_id
                    if detection.results else None)
            if kind not in features:
                continue
            center = detection.bbox.center.position
            latitude, longitude, _ = enu_2_lla(self.local_fix, center.x,
                                               center.y, center.z)
            features[kind].append(map_feature(detection, latitude, longitude))
        for kind, publisher in self.geojson_pub.items():
            publisher.publish(GeoJSON(geojson=json.dumps(
                {"type": "FeatureCollection", "features": features[kind]})))

    def status_cb(self, msg: Detection3DArray) -> None:
        markers = [clear_all(msg.header)]
        for index, detection in enumerate(msg.detections):
            status = (detection.results[0].hypothesis.class_id
                      if detection.results else "out_of_view")
            color = STATUS_COLOR.get(status, STATUS_COLOR["out_of_view"])
            position = detection.bbox.center.position
            # The gate the estimate was scored against, straight off the wire.
            markers.append(sphere(msg.header, "targets", index, position,
                                  detection.bbox.size.x, color))
            label_position = Point(x=position.x, y=position.y,
                                   z=position.z + detection.bbox.size.x / 2.0
                                   + LABEL_HEIGHT_M)
            markers.append(text(msg.header, "target_names", index,
                                label_position, detection.id))
        self.target_pub.publish(MarkerArray(markers=markers))

    def localizations_cb(self, msg: TargetBoxArray) -> None:
        """The frame's boxes, each in the color of its own verdict.

        The annotations carry the frame's stamp, not the clock's: a stamp taken
        here would slide the boxes off their frame whenever a recording is
        scrubbed.
        """
        out = ImageAnnotations()
        for index, box in enumerate(msg.uav_target_boxes):
            verdict = self.verdict_for.get(f"{box.data_source_id}_{index}")
            color = fox_color(VERDICT_COLOR.get(verdict, UNJUDGED_COLOR))
            half_w = box.target_bbox.size_x / 2.0
            half_h = box.target_bbox.size_y / 2.0
            center_x = box.target_bbox.center.position.x
            center_y = box.target_bbox.center.position.y

            outline = PointsAnnotation()
            outline.timestamp = msg.header.stamp
            outline.type = PointsAnnotation.LINE_LOOP
            outline.thickness = BOX_LINE_THICKNESS
            outline.outline_color = color
            outline.points = [
                Point2(x=center_x - half_w, y=center_y - half_h),
                Point2(x=center_x + half_w, y=center_y - half_h),
                Point2(x=center_x + half_w, y=center_y + half_h),
                Point2(x=center_x - half_w, y=center_y + half_h),
            ]
            out.points.append(outline)

            label = TextAnnotation()
            label.timestamp = msg.header.stamp
            label.position = Point2(
                x=center_x - half_w,
                y=max(center_y - half_h - 4.0, BOX_LABEL_FONT_SIZE))
            label.text = annotation_text(box.detection_class or "object",
                                         str(box.data_source_id))
            label.font_size = BOX_LABEL_FONT_SIZE
            label.text_color = fox_color(BOX_LABEL_TEXT_COLOR)
            label.background_color = fox_color(BOX_LABEL_BACKGROUND)
            out.texts.append(label)

        self.annotation_pub.publish(out)
