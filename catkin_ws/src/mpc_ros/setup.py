from setuptools import setup, find_packages

import os
from glob import glob

package_name = 'mpc_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    (f'share/{package_name}/config', ['mpc_ros/config.yaml']),
    (f'share/{package_name}/configure', ['configure/config.yaml']),
    (f'share/{package_name}/resource', ['resource/' + package_name]),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vd',
    maintainer_email='guna656565@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['mpc_Town04_launch = mpc_ros.mpc_Town04:main',
                            'carla_map_visualization = mpc_ros.carla_map_visualization:main',
                            'static_waypoint_publisher = mpc_ros.static_waypoint_publisher:main'
        ],
    },
)


