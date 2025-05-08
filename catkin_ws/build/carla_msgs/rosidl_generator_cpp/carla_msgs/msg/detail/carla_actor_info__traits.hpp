// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaActorInfo.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_ACTOR_INFO__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_ACTOR_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_actor_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaActorInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: parent_id
  {
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: rolename
  {
    out << "rolename: ";
    rosidl_generator_traits::value_to_yaml(msg.rolename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaActorInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: parent_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: rolename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rolename: ";
    rosidl_generator_traits::value_to_yaml(msg.rolename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaActorInfo & msg, bool use_flow_style = false)
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
  const carla_msgs::msg::CarlaActorInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaActorInfo & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaActorInfo>()
{
  return "carla_msgs::msg::CarlaActorInfo";
}

template<>
inline const char * name<carla_msgs::msg::CarlaActorInfo>()
{
  return "carla_msgs/msg/CarlaActorInfo";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaActorInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaActorInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::msg::CarlaActorInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_ACTOR_INFO__TRAITS_HPP_
