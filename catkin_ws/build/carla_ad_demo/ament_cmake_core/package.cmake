set(_AMENT_PACKAGE_NAME "carla_ad_demo")
set(carla_ad_demo_VERSION "0.0.1")
set(carla_ad_demo_MAINTAINER "CARLA Simulator Team <carla.simulator@gmail.com>")
set(carla_ad_demo_BUILD_DEPENDS )
set(carla_ad_demo_BUILDTOOL_DEPENDS "ament_cmake")
set(carla_ad_demo_BUILD_EXPORT_DEPENDS )
set(carla_ad_demo_BUILDTOOL_EXPORT_DEPENDS )
set(carla_ad_demo_EXEC_DEPENDS "carla_ros_bridge" "carla_spawn_objects" "carla_waypoint_publisher" "carla_ad_agent" "carla_manual_control" "rviz_carla_plugin" "carla_twist_to_control" "carla_ros_scenario_runner" "rviz2")
set(carla_ad_demo_TEST_DEPENDS )
set(carla_ad_demo_GROUP_DEPENDS )
set(carla_ad_demo_MEMBER_OF_GROUPS )
set(carla_ad_demo_DEPRECATED "")
set(carla_ad_demo_EXPORT_TAGS)
list(APPEND carla_ad_demo_EXPORT_TAGS "<build_type condition=\"$ROS_VERSION == 1\">catkin</build_type>")
list(APPEND carla_ad_demo_EXPORT_TAGS "<build_type condition=\"$ROS_VERSION == 2\">ament_cmake</build_type>")
