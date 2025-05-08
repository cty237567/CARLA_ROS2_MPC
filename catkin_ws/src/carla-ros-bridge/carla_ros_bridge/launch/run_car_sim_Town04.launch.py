from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # MPC parameter YAML file path
    mpc_config_file = os.path.join(
        get_package_share_directory('mpc_ros'),
        'configure', 'config.yaml'
    )

    # Town04 vehicle spawn definition
    spawn_config_file = os.path.join(
        get_package_share_directory('carla_spawn_objects'),
        'config', 'Town04.json'
    )

    # RViz configuration file
    rviz_config = os.path.join(
        get_package_share_directory('carla_ros_bridge'),
        'rviz', 'mpc.rviz'
    )

    config_file = os.path.join(
        get_package_share_directory('mpc_ros'),
        'configure', 'config.yaml'  # <- NOTE this path matches with actual file
    )

    return LaunchDescription([
        # ------------- Declare Launch Args -------------
        DeclareLaunchArgument('host', default_value='localhost'),
        DeclareLaunchArgument('port', default_value='2000'),
        DeclareLaunchArgument('timeout', default_value='200'),
        DeclareLaunchArgument('passive', default_value='False'),
        DeclareLaunchArgument('synchronous_mode', default_value='True'),
        DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='False'),
        DeclareLaunchArgument('fixed_delta_seconds', default_value='0.1'),
        DeclareLaunchArgument('town', default_value='Town04'),
        DeclareLaunchArgument('spawn_point', default_value=''),
        DeclareLaunchArgument('ego_vehicle_role_name', default_value='["hero", "agent_0"]'),
        DeclareLaunchArgument('auto_control', default_value='False'),

        # ------------- ROS Bridge Node -------------
        Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'timeout': LaunchConfiguration('timeout'),
                'passive': LaunchConfiguration('passive'),
                'synchronous_mode': LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds'),
                'town': LaunchConfiguration('town'),
                'register_all_sensors': False,
                'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
                'publish_tf': True
            }]
        ),

        # ------------- Spawn Agent + Sensors -------------
        Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            parameters=[{
                'objects_definition_file': spawn_config_file
            }]
        ),

        # ------------- MPC Map Visualizer -------------
        Node(
            package='mpc_ros',
            executable='carla_map_visualization',
            name='carla_map_visualization',
            output='screen'
        ),

        # Launch the CARLA Waypoint Publisher (ROS2)
        Node(
            package='carla_waypoint_publisher',
            executable='carla_waypoint_publisher',
            name='carla_waypoint_publisher',
            output='screen',
            parameters=[{
                'role_name': 'agent_0'
            }]
        ),

        

        # ------------- Manual Control (Optional for Debugging) -------------
        Node(
            package='carla_manual_control',
            executable='carla_manual_control',
            name='carla_manual_control',
            output='screen',
            parameters=[{'role_name': 'agent_0'}]
            
        ),

        Node(
            package='mpc_ros',
            executable='static_waypoint_publisher',
            name='waypoint_publisher',
            output='screen'
        ),

                

        # ------------- RViz -------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
