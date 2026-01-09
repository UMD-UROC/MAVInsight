import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mavinsight"
runtime_resource_folder = f"share/{package_name}/package_resources"
vehicle_configs = [f for f in glob(os.path.join("vehicles", "**/*"), recursive=True) if os.path.isfile(f)]
sensor_configs = [f for f in glob(os.path.join("sensors", "**/*"), recursive=True) if os.path.isfile(f)]
resource_configs = [f for f in glob(os.path.join("resource", "**/*"), recursive=True) if os.path.isfile(f)]
site_configs = [f for f in glob(os.path.join("sites", "**/**"), recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"],
        include=["mavinsight", "mavinsight.*", "models", "models.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ['resource/' + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (runtime_resource_folder, vehicle_configs),
        (runtime_resource_folder, sensor_configs),
        (runtime_resource_folder, resource_configs),
        (runtime_resource_folder, site_configs),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cdenihan",
    maintainer_email="cdenihan@proton.me",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "graph_member = models.graph_member:GraphMember.main",
            "vehicle = models.vehicle:Vehicle.main",
            "sensor = models.sensor:Sensor.main",
            "camera = models.sensor:Camera.main",
            "gimbal = models.sensor:Gimbal.main",
            "rangefinder = models.sensor:Rangefinder.main",
            "site = models.site:Site.main"
        ],
    },
)
