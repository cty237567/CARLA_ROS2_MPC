// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaEgoVehicleInfoWheel.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO_WHEEL__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO_WHEEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_ego_vehicle_info_wheel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaEgoVehicleInfoWheel & msg,
  std::ostream & out)
{
  out << "{";
  // member: tire_friction
  {
    out << "tire_friction: ";
    rosidl_generator_traits::value_to_yaml(msg.tire_friction, out);
    out << ", ";
  }

  // member: damping_rate
  {
    out << "damping_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate, out);
    out << ", ";
  }

  // member: max_steer_angle
  {
    out << "max_steer_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.max_steer_angle, out);
    out << ", ";
  }

  // member: radius
  {
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << ", ";
  }

  // member: max_brake_torque
  {
    out << "max_brake_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.max_brake_torque, out);
    out << ", ";
  }

  // member: max_handbrake_torque
  {
    out << "max_handbrake_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.max_handbrake_torque, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaEgoVehicleInfoWheel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tire_friction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tire_friction: ";
    rosidl_generator_traits::value_to_yaml(msg.tire_friction, out);
    out << "\n";
  }

  // member: damping_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate, out);
    out << "\n";
  }

  // member: max_steer_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_steer_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.max_steer_angle, out);
    out << "\n";
  }

  // member: radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << "\n";
  }

  // member: max_brake_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_brake_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.max_brake_torque, out);
    out << "\n";
  }

  // member: max_handbrake_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_handbrake_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.max_handbrake_torque, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaEgoVehicleInfoWheel & msg, bool use_flow_style = false)
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
  const carla_msgs::msg::CarlaEgoVehicleInfoWheel & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaEgoVehicleInfoWheel & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaEgoVehicleInfoWheel>()
{
  return "carla_msgs::msg::CarlaEgoVehicleInfoWheel";
}

template<>
inline const char * name<carla_msgs::msg::CarlaEgoVehicleInfoWheel>()
{
  return "carla_msgs/msg/CarlaEgoVehicleInfoWheel";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaEgoVehicleInfoWheel>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaEgoVehicleInfoWheel>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<carla_msgs::msg::CarlaEgoVehicleInfoWheel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO_WHEEL__TRAITS_HPP_
