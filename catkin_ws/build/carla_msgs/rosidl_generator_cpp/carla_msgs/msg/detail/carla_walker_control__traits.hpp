// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaWalkerControl.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_WALKER_CONTROL__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_WALKER_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_walker_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaWalkerControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction
  {
    out << "direction: ";
    to_flow_style_yaml(msg.direction, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: jump
  {
    out << "jump: ";
    rosidl_generator_traits::value_to_yaml(msg.jump, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaWalkerControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction:\n";
    to_block_style_yaml(msg.direction, out, indentation + 2);
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: jump
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jump: ";
    rosidl_generator_traits::value_to_yaml(msg.jump, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaWalkerControl & msg, bool use_flow_style = false)
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

}  // namespace carla_msgs

namespace rosidl_generator_traits
{

[[deprecated("use carla_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_msgs::msg::CarlaWalkerControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaWalkerControl & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaWalkerControl>()
{
  return "carla_msgs::msg::CarlaWalkerControl";
}

template<>
inline const char * name<carla_msgs::msg::CarlaWalkerControl>()
{
  return "carla_msgs/msg/CarlaWalkerControl";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaWalkerControl>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaWalkerControl>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<carla_msgs::msg::CarlaWalkerControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_WALKER_CONTROL__TRAITS_HPP_
