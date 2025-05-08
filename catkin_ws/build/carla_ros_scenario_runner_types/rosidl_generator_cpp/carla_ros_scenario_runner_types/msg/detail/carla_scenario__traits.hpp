// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ros_scenario_runner_types:msg/CarlaScenario.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO__TRAITS_HPP_
#define CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_ros_scenario_runner_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaScenario & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: scenario_file
  {
    out << "scenario_file: ";
    rosidl_generator_traits::value_to_yaml(msg.scenario_file, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaScenario & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: scenario_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scenario_file: ";
    rosidl_generator_traits::value_to_yaml(msg.scenario_file, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaScenario & msg, bool use_flow_style = false)
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

}  // namespace carla_ros_scenario_runner_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_ros_scenario_runner_types::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_ros_scenario_runner_types::msg::CarlaScenario & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ros_scenario_runner_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ros_scenario_runner_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ros_scenario_runner_types::msg::CarlaScenario & msg)
{
  return carla_ros_scenario_runner_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ros_scenario_runner_types::msg::CarlaScenario>()
{
  return "carla_ros_scenario_runner_types::msg::CarlaScenario";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::msg::CarlaScenario>()
{
  return "carla_ros_scenario_runner_types/msg/CarlaScenario";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::msg::CarlaScenario>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::msg::CarlaScenario>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_ros_scenario_runner_types::msg::CarlaScenario>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO__TRAITS_HPP_
