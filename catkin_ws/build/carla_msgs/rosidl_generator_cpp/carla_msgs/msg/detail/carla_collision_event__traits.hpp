// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaCollisionEvent.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_COLLISION_EVENT__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_COLLISION_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_collision_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'normal_impulse'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaCollisionEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: other_actor_id
  {
    out << "other_actor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.other_actor_id, out);
    out << ", ";
  }

  // member: normal_impulse
  {
    out << "normal_impulse: ";
    to_flow_style_yaml(msg.normal_impulse, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaCollisionEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: other_actor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "other_actor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.other_actor_id, out);
    out << "\n";
  }

  // member: normal_impulse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "normal_impulse:\n";
    to_block_style_yaml(msg.normal_impulse, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaCollisionEvent & msg, bool use_flow_style = false)
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
  const carla_msgs::msg::CarlaCollisionEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaCollisionEvent & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaCollisionEvent>()
{
  return "carla_msgs::msg::CarlaCollisionEvent";
}

template<>
inline const char * name<carla_msgs::msg::CarlaCollisionEvent>()
{
  return "carla_msgs/msg/CarlaCollisionEvent";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaCollisionEvent>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaCollisionEvent>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<carla_msgs::msg::CarlaCollisionEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_COLLISION_EVENT__TRAITS_HPP_
