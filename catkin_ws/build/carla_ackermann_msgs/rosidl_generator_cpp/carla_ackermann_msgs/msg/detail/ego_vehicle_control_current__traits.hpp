// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlCurrent.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_CURRENT__TRAITS_HPP_
#define CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_CURRENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_current__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_ackermann_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EgoVehicleControlCurrent & msg,
  std::ostream & out)
{
  out << "{";
  // member: time_sec
  {
    out << "time_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_sec, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: speed_abs
  {
    out << "speed_abs: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_abs, out);
    out << ", ";
  }

  // member: accel
  {
    out << "accel: ";
    rosidl_generator_traits::value_to_yaml(msg.accel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EgoVehicleControlCurrent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.time_sec, out);
    out << "\n";
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

  // member: speed_abs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_abs: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_abs, out);
    out << "\n";
  }

  // member: accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel: ";
    rosidl_generator_traits::value_to_yaml(msg.accel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EgoVehicleControlCurrent & msg, bool use_flow_style = false)
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

}  // namespace carla_ackermann_msgs

namespace rosidl_generator_traits
{

[[deprecated("use carla_ackermann_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_ackermann_msgs::msg::EgoVehicleControlCurrent & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ackermann_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ackermann_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ackermann_msgs::msg::EgoVehicleControlCurrent & msg)
{
  return carla_ackermann_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>()
{
  return "carla_ackermann_msgs::msg::EgoVehicleControlCurrent";
}

template<>
inline const char * name<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>()
{
  return "carla_ackermann_msgs/msg/EgoVehicleControlCurrent";
}

template<>
struct has_fixed_size<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_CURRENT__TRAITS_HPP_
