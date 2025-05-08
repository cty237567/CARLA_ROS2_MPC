from setuptools import find_packages
from setuptools import setup

setup(
    name='carla_waypoint_types',
    version='0.1.0',
    packages=find_packages(
        include=('carla_waypoint_types', 'carla_waypoint_types.*')),
)
