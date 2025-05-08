from setuptools import find_packages
from setuptools import setup

setup(
    name='carla_ackermann_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('carla_ackermann_msgs', 'carla_ackermann_msgs.*')),
)
