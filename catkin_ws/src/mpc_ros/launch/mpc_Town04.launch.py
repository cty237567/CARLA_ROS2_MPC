from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mpc_ros'),
        'configure', 'config.yaml'  # <- NOTE this path should match with actual file
    )

    return LaunchDescription([
        # MPC controller node
        Node(
            package='mpc_ros',
            executable='static_waypoint_publisher',
            name='waypoint_publisher',
            output='screen'
        ),

        Node(
            package='mpc_ros',
            executable='mpc_Town04_launch',
            name='mpc_node',
            output='screen',
            parameters=['/home/vd/carla-ros-bridge/catkin_ws/install/mpc_ros/share/mpc_ros/configure/config.yaml']
        )
    ])