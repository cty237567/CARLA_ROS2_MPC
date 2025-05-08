// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlStatus.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__TRAITS_HPP_
#define CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_ackermann_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EgoVehicleControlStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: speed_control_activation_count
  {
    out << "speed_control_activation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_activation_count, out);
    out << ", ";
  }

  // member: speed_control_accel_delta
  {
    out << "speed_control_accel_delta: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_accel_delta, out);
    out << ", ";
  }

  // member: speed_control_accel_target
  {
    out << "speed_control_accel_target: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_accel_target, out);
    out << ", ";
  }

  // member: accel_control_pedal_delta
  {
    out << "accel_control_pedal_delta: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_control_pedal_delta, out);
    out << ", ";
  }

  // member: accel_control_pedal_target
  {
    out << "accel_control_pedal_target: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_control_pedal_target, out);
    out << ", ";
  }

  // member: brake_upper_border
  {
    out << "brake_upper_border: ";
    rosidl_generator_traits::value_to_yaml(msg.brake_upper_border, out);
    out << ", ";
  }

  // member: throttle_lower_border
  {
    out << "throttle_lower_border: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle_lower_border, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EgoVehicleControlStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: speed_control_activation_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_control_activation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_activation_count, out);
    out << "\n";
  }

  // member: speed_control_accel_delta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_control_accel_delta: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_accel_delta, out);
    out << "\n";
  }

  // member: speed_control_accel_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_control_accel_target: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_control_accel_target, out);
    out << "\n";
  }

  // member: accel_control_pedal_delta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_control_pedal_delta: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_control_pedal_delta, out);
    out << "\n";
  }

  // member: accel_control_pedal_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_control_pedal_target: ";
    rosidl_generator_traits::value_to_yaml(msg.accel_control_pedal_target, out);
    out << "\n";
  }

  // member: brake_upper_border
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake_upper_border: ";
    rosidl_generator_traits::value_to_yaml(msg.brake_upper_border, out);
    out << "\n";
  }

  // member: throttle_lower_border
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle_lower_border: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle_lower_border, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EgoVehicleControlStatus & msg, bool use_flow_style = false)
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
  const carla_ackermann_msgs::msg::EgoVehicleControlStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ackermann_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ackermann_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ackermann_msgs::msg::EgoVehicleControlStatus & msg)
{
  return carla_ackermann_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ackermann_msgs::msg::EgoVehicleControlStatus>()
{
  return "carla_ackermann_msgs::msg::EgoVehicleControlStatus";
}

template<>
inline const char * name<carla_ackermann_msgs::msg::EgoVehicleControlStatus>()
{
  return "carla_ackermann_msgs/msg/EgoVehicleControlStatus";
}

template<>
struct has_fixed_size<carla_ackermann_msgs::msg::EgoVehicleControlStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_ackermann_msgs::msg::EgoVehicleControlStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_ackermann_msgs::msg::EgoVehicleControlStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__TRAITS_HPP_
