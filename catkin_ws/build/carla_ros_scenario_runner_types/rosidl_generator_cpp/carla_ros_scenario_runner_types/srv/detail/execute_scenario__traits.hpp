// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_ros_scenario_runner_types:srv/ExecuteScenario.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ROS_SCENARIO_RUNNER_TYPES__SRV__DETAIL__EXECUTE_SCENARIO__TRAITS_HPP_
#define CARLA_ROS_SCENARIO_RUNNER_TYPES__SRV__DETAIL__EXECUTE_SCENARIO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_ros_scenario_runner_types/srv/detail/execute_scenario__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'scenario'
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario__traits.hpp"

namespace carla_ros_scenario_runner_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteScenario_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: scenario
  {
    out << "scenario: ";
    to_flow_style_yaml(msg.scenario, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteScenario_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: scenario
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scenario:\n";
    to_block_style_yaml(msg.scenario, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteScenario_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace carla_ros_scenario_runner_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_ros_scenario_runner_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_ros_scenario_runner_types::srv::ExecuteScenario_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ros_scenario_runner_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ros_scenario_runner_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_ros_scenario_runner_types::srv::ExecuteScenario_Request & msg)
{
  return carla_ros_scenario_runner_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>()
{
  return "carla_ros_scenario_runner_types::srv::ExecuteScenario_Request";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>()
{
  return "carla_ros_scenario_runner_types/srv/ExecuteScenario_Request";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>
  : std::integral_constant<bool, has_fixed_size<carla_ros_scenario_runner_types::msg::CarlaScenario>::value> {};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>
  : std::integral_constant<bool, has_bounded_size<carla_ros_scenario_runner_types::msg::CarlaScenario>::value> {};

template<>
struct is_message<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace carla_ros_scenario_runner_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteScenario_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteScenario_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteScenario_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace carla_ros_scenario_runner_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_ros_scenario_runner_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_ros_scenario_runner_types::srv::ExecuteScenario_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_ros_scenario_runner_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_ros_scenario_runner_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_ros_scenario_runner_types::srv::ExecuteScenario_Response & msg)
{
  return carla_ros_scenario_runner_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>()
{
  return "carla_ros_scenario_runner_types::srv::ExecuteScenario_Response";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>()
{
  return "carla_ros_scenario_runner_types/srv/ExecuteScenario_Response";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_ros_scenario_runner_types::srv::ExecuteScenario>()
{
  return "carla_ros_scenario_runner_types::srv::ExecuteScenario";
}

template<>
inline const char * name<carla_ros_scenario_runner_types::srv::ExecuteScenario>()
{
  return "carla_ros_scenario_runner_types/srv/ExecuteScenario";
}

template<>
struct has_fixed_size<carla_ros_scenario_runner_types::srv::ExecuteScenario>
  : std::integral_constant<
    bool,
    has_fixed_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>::value &&
    has_fixed_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>::value
  >
{
};

template<>
struct has_bounded_size<carla_ros_scenario_runner_types::srv::ExecuteScenario>
  : std::integral_constant<
    bool,
    has_bounded_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>::value &&
    has_bounded_size<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>::value
  >
{
};

template<>
struct is_service<carla_ros_scenario_runner_types::srv::ExecuteScenario>
  : std::true_type
{
};

template<>
struct is_service_request<carla_ros_scenario_runner_types::srv::ExecuteScenario_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carla_ros_scenario_runner_types::srv::ExecuteScenario_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARLA_ROS_SCENARIO_RUNNER_TYPES__SRV__DETAIL__EXECUTE_SCENARIO__TRAITS_HPP_
