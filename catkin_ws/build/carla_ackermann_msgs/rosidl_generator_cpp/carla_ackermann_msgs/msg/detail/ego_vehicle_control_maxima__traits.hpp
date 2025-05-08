// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlMaxima.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_MAXIMA__TRAITS_HPP_
#define CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_MAXIMA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_maxima__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_ackermann_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EgoVehicleControlMaxima & msg,
  std::ostream & out)
{
  out << "{";
  // member: max_steering_angle
  {
    out << "max_steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.max_steering_angle, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << ", ";
  }

  // member: max_accel
  {
    out << "max_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_accel, out);
    out << ", ";
  }

  // member: max_decel
  {
    out << "max_decel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_decel, out);
    out << ", ";
  }

  // member: min_accel
  {
    out << "min_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.min_accel, out);
    out << ", ";
  }

  // member: max_pedal
  {
    out << "max_pedal: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pedal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EgoVehicleControlMaxima & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: max_steering_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.max_steering_angle, out);
    out << "\n";
  }

  // member: max_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << "\n";
  }

  // member: max_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_accel, out);
    out << "\n";
  }

  // member: max_decel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_decel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_decel, out);
    out << "\n";
  }

  // member: min_accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_accel: ";
    rosidl_generator_traits::value_to_yaml(msg.min_accel, out);
    out << "\n";
  }

  // member: max_pedal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_pedal: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pedal, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EgoVehicleControlMaxima & msg, bool use_flow_style = false)
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
  const carla_ackermann_msgs::msg::EgoVehicleControlMaxima & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ackermann_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ackermann_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ackermann_msgs::msg::EgoVehicleControlMaxima & msg)
{
  return carla_ackermann_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ackermann_msgs::msg::EgoVehicleControlMaxima>()
{
  return "carla_ackermann_msgs::msg::EgoVehicleControlMaxima";
}

template<>
inline const char * name<carla_ackermann_msgs::msg::EgoVehicleControlMaxima>()
{
  return "carla_ackermann_msgs/msg/EgoVehicleControlMaxima";
}

template<>
struct has_fixed_size<carla_ackermann_msgs::msg::EgoVehicleControlMaxima>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_ackermann_msgs::msg::EgoVehicleControlMaxima>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_ackermann_msgs::msg::EgoVehicleControlMaxima>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_MAXIMA__TRAITS_HPP_
