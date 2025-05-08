// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ros_scenario_runner_types:msg/CarlaScenarioRunnerStatus.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_RUNNER_STATUS__TRAITS_HPP_
#define CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_RUNNER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario_runner_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_ros_scenario_runner_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaScenarioRunnerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaScenarioRunnerStatus & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaScenarioRunnerStatus & msg, bool use_flow_style = false)
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
  const carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ros_scenario_runner_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ros_scenario_runner_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus & msg)
{
  return carla_ros_scenario_runner_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>()
{
  return "carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>()
{
  return "carla_ros_scenario_runner_types/msg/CarlaScenarioRunnerStatus";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_ros_scenario_runner_types::msg::CarlaScenarioRunnerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_RUNNER_STATUS__TRAITS_HPP_
