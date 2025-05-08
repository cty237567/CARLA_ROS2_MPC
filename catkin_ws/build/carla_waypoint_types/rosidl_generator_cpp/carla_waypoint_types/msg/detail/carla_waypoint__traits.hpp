// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_waypoint_types:msg/CarlaWaypoint.idl
// generated code does not contain a copyright notice

#ifndef CARLA_WAYPOINT_TYPES__MSG__DETAIL__CARLA_WAYPOINT__TRAITS_HPP_
#define CARLA_WAYPOINT_TYPES__MSG__DETAIL__CARLA_WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_waypoint_types/msg/detail/carla_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace carla_waypoint_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaWaypoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: road_id
  {
    out << "road_id: ";
    rosidl_generator_traits::value_to_yaml(msg.road_id, out);
    out << ", ";
  }

  // member: section_id
  {
    out << "section_id: ";
    rosidl_generator_traits::value_to_yaml(msg.section_id, out);
    out << ", ";
  }

  // member: lane_id
  {
    out << "lane_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_id, out);
    out << ", ";
  }

  // member: is_junction
  {
    out << "is_junction: ";
    rosidl_generator_traits::value_to_yaml(msg.is_junction, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: road_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "road_id: ";
    rosidl_generator_traits::value_to_yaml(msg.road_id, out);
    out << "\n";
  }

  // member: section_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "section_id: ";
    rosidl_generator_traits::value_to_yaml(msg.section_id, out);
    out << "\n";
  }

  // member: lane_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lane_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lane_id, out);
    out << "\n";
  }

  // member: is_junction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_junction: ";
    rosidl_generator_traits::value_to_yaml(msg.is_junction, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaWaypoint & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace carla_waypoint_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_waypoint_types::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_waypoint_types::msg::CarlaWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_waypoint_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_waypoint_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_waypoint_types::msg::CarlaWaypoint & msg)
{
  return carla_waypoint_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_waypoint_types::msg::CarlaWaypoint>()
{
  return "carla_waypoint_types::msg::CarlaWaypoint";
}

template<>
inline const char * name<carla_waypoint_types::msg::CarlaWaypoint>()
{
  return "carla_waypoint_types/msg/CarlaWaypoint";
}

template<>
struct has_fixed_size<carla_waypoint_types::msg::CarlaWaypoint>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<carla_waypoint_types::msg::CarlaWaypoint>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<carla_waypoint_types::msg::CarlaWaypoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_WAYPOINT_TYPES__MSG__DETAIL__CARLA_WAYPOINT__TRAITS_HPP_
