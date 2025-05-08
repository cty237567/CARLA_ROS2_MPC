// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_waypoint_types:srv/GetActorWaypoint.idl
// generated code does not contain a copyright notice

#ifndef CARLA_WAYPOINT_TYPES__SRV__DETAIL__GET_ACTOR_WAYPOINT__TRAITS_HPP_
#define CARLA_WAYPOINT_TYPES__SRV__DETAIL__GET_ACTOR_WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_waypoint_types/srv/detail/get_actor_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_waypoint_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetActorWaypoint_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetActorWaypoint_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetActorWaypoint_Request & msg, bool use_flow_style = false)
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

}  // namespace carla_waypoint_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_waypoint_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_waypoint_types::srv::GetActorWaypoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_waypoint_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_waypoint_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_waypoint_types::srv::GetActorWaypoint_Request & msg)
{
  return carla_waypoint_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_waypoint_types::srv::GetActorWaypoint_Request>()
{
  return "carla_waypoint_types::srv::GetActorWaypoint_Request";
}

template<>
inline const char * name<carla_waypoint_types::srv::GetActorWaypoint_Request>()
{
  return "carla_waypoint_types/srv/GetActorWaypoint_Request";
}

template<>
struct has_fixed_size<carla_waypoint_types::srv::GetActorWaypoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_waypoint_types::srv::GetActorWaypoint_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_waypoint_types::srv::GetActorWaypoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'waypoint'
#include "carla_waypoint_types/msg/detail/carla_waypoint__traits.hpp"

namespace carla_waypoint_types
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetActorWaypoint_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: waypoint
  {
    out << "waypoint: ";
    to_flow_style_yaml(msg.waypoint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetActorWaypoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: waypoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoint:\n";
    to_block_style_yaml(msg.waypoint, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetActorWaypoint_Response & msg, bool use_flow_style = false)
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

}  // namespace carla_waypoint_types

namespace rosidl_generator_traits
{

[[deprecated("use carla_waypoint_types::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_waypoint_types::srv::GetActorWaypoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_waypoint_types::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_waypoint_types::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_waypoint_types::srv::GetActorWaypoint_Response & msg)
{
  return carla_waypoint_types::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_waypoint_types::srv::GetActorWaypoint_Response>()
{
  return "carla_waypoint_types::srv::GetActorWaypoint_Response";
}

template<>
inline const char * name<carla_waypoint_types::srv::GetActorWaypoint_Response>()
{
  return "carla_waypoint_types/srv/GetActorWaypoint_Response";
}

template<>
struct has_fixed_size<carla_waypoint_types::srv::GetActorWaypoint_Response>
  : std::integral_constant<bool, has_fixed_size<carla_waypoint_types::msg::CarlaWaypoint>::value> {};

template<>
struct has_bounded_size<carla_waypoint_types::srv::GetActorWaypoint_Response>
  : std::integral_constant<bool, has_bounded_size<carla_waypoint_types::msg::CarlaWaypoint>::value> {};

template<>
struct is_message<carla_waypoint_types::srv::GetActorWaypoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_waypoint_types::srv::GetActorWaypoint>()
{
  return "carla_waypoint_types::srv::GetActorWaypoint";
}

template<>
inline const char * name<carla_waypoint_types::srv::GetActorWaypoint>()
{
  return "carla_waypoint_types/srv/GetActorWaypoint";
}

template<>
struct has_fixed_size<carla_waypoint_types::srv::GetActorWaypoint>
  : std::integral_constant<
    bool,
    has_fixed_size<carla_waypoint_types::srv::GetActorWaypoint_Request>::value &&
    has_fixed_size<carla_waypoint_types::srv::GetActorWaypoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<carla_waypoint_types::srv::GetActorWaypoint>
  : std::integral_constant<
    bool,
    has_bounded_size<carla_waypoint_types::srv::GetActorWaypoint_Request>::value &&
    has_bounded_size<carla_waypoint_types::srv::GetActorWaypoint_Response>::value
  >
{
};

template<>
struct is_service<carla_waypoint_types::srv::GetActorWaypoint>
  : std::true_type
{
};

template<>
struct is_service_request<carla_waypoint_types::srv::GetActorWaypoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carla_waypoint_types::srv::GetActorWaypoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARLA_WAYPOINT_TYPES__SRV__DETAIL__GET_ACTOR_WAYPOINT__TRAITS_HPP_
