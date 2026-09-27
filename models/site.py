# python imports
import json
import os
from pathlib import Path
import time

import numpy as np

# ROS2 message imports
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

# MAVInsight imports
from models.frame_utils import lla_2_enu
from models.graph_member import GraphMember
from models.qos_profiles import reliable_qos, viz_qos
from models.scene_ground import TerrainSurface

try:
    from foxglove_msgs.msg import GeoJSON
except ImportError:
    GeoJSON = None


def param(node, name, default):
    """Read an optional parameter without making old site configs noisy."""
    return node.get_parameter(name).value if node.has_parameter(name) else default

class Site(GraphMember):

    GEOFENCE_TOPIC: str
    GT_TOPIC: str
    LOCAL_FRAME: str
    MAP_REF_TOPIC: str
    NAME: str

    def __init__(self):
        super().__init__()
        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Site params...")

        self.LOCAL_FIX = None

        # A generated scenegen scene is the authoritative source for these
        # display-only boundaries. Keep the old flat geofence parameters below
        # for existing sites that do not have a scene file.
        scene_file = param(self, "scene_file", "")
        surface_file = param(self, "terrain_surface_file", "")
        if not scene_file and os.environ.get("SCENE"):
            scene_name = os.environ["SCENE"]
            candidates = (
                Path(f"/home/user/px4-sim-stack/modules/scenegen/data/{scene_name}/scene.json"),
                Path(f"/scenegen/data/{scene_name}/scene.json"),
            )
            scene_file = next((str(path) for path in candidates if path.is_file()), "")
            if not surface_file:
                surface = Path(f"/home/user/px4-sim-stack/modules/sim/scenes/worlds/{scene_name}_surface.json")
                if surface.is_file():
                    surface_file = str(surface)
        self.scene = None
        self.surface = None
        if scene_file:
            try:
                self.scene = json.loads(Path(scene_file).read_text())
                self.surface = TerrainSurface.load(surface_file)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self.get_logger().warning(f"cannot read scene display data: {error}")

        # geofence
        if self.has_parameter("geofence_topic"):
            self.GEOFENCE_TOPIC = self.get_parameter("geofence_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("geofence_topic")
            self.GEOFENCE_TOPIC = "/geofence"

        if self.has_parameter("geofence"):
            self.geofence = self.get_parameter("geofence").get_parameter_value().double_array_value
        else:
            self.geofence = None

        # GTs
        if self.has_parameter("ground_truth_topic"):
            self.GT_TOPIC = self.get_parameter("ground_truth_topic").get_parameter_value().string_value
        else:
            self.default_parameter_warning("ground_truth_topic")
            self.GT_TOPIC = "/ground_truths"

        if self.has_parameter("ground_truths"):
            self.ground_truths = self.get_parameter("ground_truths").get_parameter_value().double_array_value
        else:
            self.ground_truths = None

        # local fix
        if self.has_parameter("local_fix_topic"):
            local_fix_topic = self.get_parameter("local_fix_topic").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Site Node: {self.DISPLAY_NAME} local fix param not set. Unable to initialize Site node.")

        if self.has_parameter("local_frame"):
            self.LOCAL_FRAME = self.get_parameter("local_frame").get_parameter_value().string_value
        else:
            raise RuntimeError(f"Site Node: {self.DISPLAY_NAME} local frame not set. Unable to initialize Site node.")

        if self.has_parameter("name"):
            self.NAME = self.get_parameter("name").get_parameter_value().string_value
        else:
            self.default_parameter_warning("name")
            self.NAME = "site"

        self.timer = self.create_timer(3, self.site_foxglove_loiter)

        self.create_subscription(NavSatFix, local_fix_topic, self.update_local_fix, viz_qos)

        self.geofence_pub = self.create_publisher(Marker, self.GEOFENCE_TOPIC, reliable_qos)
        self.exclusion_pub = self.create_publisher(
            Marker, param(self, "exclusion_zone_topic", "/exclusion_zones"), reliable_qos)
        self.geojson_pub = (self.create_publisher(
            GeoJSON, param(self, "site_geojson_topic", "/viz/site_zones"), reliable_qos)
            if GeoJSON is not None else None)
        self.gt_pub = self.create_publisher(Marker, f"/{self.NAME}{self.GT_TOPIC}", reliable_qos)

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Site Initialized...")

    def update_local_fix(self, msg: NavSatFix):
        self.LOCAL_FIX = msg

    def site_foxglove_loiter(self):
        if not self.LOCAL_FIX:
            return

        if self.geofence:
            if len(self.geofence) % 2 != 0:
                raise ValueError(
                    f"Non-even geofence list length. Ensure all lats and lons are paired.\n" +
                    f"length: {len(self.geofence)}"
                )
            vertices = list(zip(self.geofence[0::2], self.geofence[1::2]))
            # close the loop
            if vertices[0] != vertices[-1]:
                vertices.append(vertices[0])
            vertices = [lla_2_enu(self.LOCAL_FIX, NavSatFix(latitude=lat, longitude=lon)) for lat, lon in vertices]
            geofence_points = [Point(x=e, y=n, z=u) for e, n, u in vertices]
            self.geofence_pub.publish(Marker(
                header=Header(frame_id=self.LOCAL_FRAME),
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                points=geofence_points,
                scale=Vector3(x=0.1, y=0.1, z=1.0),
                color=ColorRGBA(r=253.0/256.0, g=138.0/256.0, a=1.0)
            ))

        if self.scene:
            self.publish_scene_zones()

        if self.ground_truths:
            gt_coords = list(zip(self.ground_truths[0::2], self.ground_truths[1::2]))
            gt_fixes = [lla_2_enu(self.LOCAL_FIX, NavSatFix(latitude=lat, longitude=lon)) for lat, lon in gt_coords]
            gt_msg_points = [Point(x=e, y=n, z=u) for e, n, u in gt_fixes]

            self.gt_pub.publish(Marker(
                header=Header(frame_id=self.LOCAL_FRAME),
                type=Marker.POINTS,
                action=Marker.ADD,
                points=gt_msg_points,
                scale=Vector3(x=1.0, y=1.0, z=1.0),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            ))

    def publish_scene_zones(self):
        """Publish imported scene boundaries, draped over the terrain grid."""
        origin = self.scene.get("center_lat"), self.scene.get("center_lon")
        origin_alt = float(self.scene.get("origin_alt_m", 0.0))
        geoid = float(self.get_parameter("geoid_height_m").value) \
            if self.has_parameter("geoid_height_m") else 0.0
        correction = np.zeros(3)
        features = []
        for key, publisher, color, label in (
                ("geofences", self.geofence_pub, (0.15, 0.55, 1.0, 0.9), "geofence"),
                ("exclusion_zones", self.exclusion_pub, (1.0, 0.55, 0.1, 0.9), "exclusion zone")):
            for index, zone in enumerate(self.scene.get(key, [])):
                ring = zone.get("polygon_m", [])
                if len(ring) < 3 or not zone.get("enabled", True):
                    continue
                fixes = []
                for east, north in ring:
                    height = self.surface.height(east, north) if self.surface else 0.0
                    lat, lon = self.enu_to_latlon(origin[0], origin[1], east, north)
                    fixes.append(NavSatFix(latitude=lat, longitude=lon,
                                           altitude=origin_alt + height + geoid))
                enu = [lla_2_enu(self.LOCAL_FIX, fix, ignore_alt=False)
                       for fix in fixes]
                points = [Point(x=e-correction[0], y=n-correction[1], z=u-correction[2])
                          for e, n, u in enu]
                points.append(points[0])
                publisher.publish(Marker(
                    header=Header(frame_id=self.LOCAL_FRAME), type=Marker.LINE_STRIP,
                    action=Marker.ADD, id=index, ns=label, points=points,
                    scale=Vector3(x=0.2), color=ColorRGBA(*color)))
                features.append({"type": "Feature",
                                 "geometry": {"type": "LineString",
                                              "coordinates": [[f.longitude, f.latitude]
                                                              for f in fixes] +
                                                             [[fixes[0].longitude,
                                                               fixes[0].latitude]]},
                                 "properties": {"name": f"{label} {index + 1}",
                                                "style": {"weight": 2}}})
        if self.geojson_pub is not None:
            self.geojson_pub.publish(GeoJSON(geojson=json.dumps(
                {"type": "FeatureCollection", "features": features})))

    @staticmethod
    def enu_to_latlon(lat, lon, east, north):
        """Small-scene ENU inverse, sufficient for scene boundary vertices."""
        import math
        return (lat + north / 111320.0,
                lon + east / (111320.0 * math.cos(math.radians(lat))))
