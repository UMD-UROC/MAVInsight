import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mavinsight'
vehicle_configs = glob(os.path.join('vehicles','*'))
sensor_configs = glob(os.path.join('sensors', '*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'],
                           include=['mavinsight', 'mavinsight.*',
                                    'models', 'models.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/vehicles', vehicle_configs),
        (f'share/{package_name}/sensors', sensor_configs),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdenihan',
    maintainer_email='cdenihan@proton.me',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vehicle_tf_publisher = mavinsight.vehicle_tf_publisher:main"
        ],
    },
)
