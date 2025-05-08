// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ros_scenario_runner_types:msg/CarlaScenarioList.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_LIST__TRAITS_HPP_
#define CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'scenarios'
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario__traits.hpp"

namespace carla_ros_scenario_runner_types
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaScenarioList & msg,
  std::ostream & out)
{
  out << "{";
  // member: scenarios
  {
    if (msg.scenarios.size() == 0) {
      out << "scenarios: []";
    } else {
      out << "scenarios: [";
      size_t pending_items = msg.scenarios.size();
      for (auto item : msg.scenarios) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaScenarioList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: scenarios
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scenarios.size() == 0) {
      out << "scenarios: []\n";
    } else {
      out << "scenarios:\n";
      for (auto item : msg.scenarios) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaScenarioList & msg, bool use_flow_style = false)
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
  const carla_ros_scenario_runner_types::msg::CarlaScenarioList & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ros_scenario_runner_types::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ros_scenario_runner_types::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_ros_scenario_runner_types::msg::CarlaScenarioList & msg)
{
  return carla_ros_scenario_runner_types::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ros_scenario_runner_types::msg::CarlaScenarioList>()
{
  return "carla_ros_scenario_runner_types::msg::CarlaScenarioList";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::msg::CarlaScenarioList>()
{
  return "carla_ros_scenario_runner_types/msg/CarlaScenarioList";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::msg::CarlaScenarioList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::msg::CarlaScenarioList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_ros_scenario_runner_types::msg::CarlaScenarioList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ROS_SCENARIO_RUNNER_TYPES__MSG__DETAIL__CARLA_SCENARIO_LIST__TRAITS_HPP_
