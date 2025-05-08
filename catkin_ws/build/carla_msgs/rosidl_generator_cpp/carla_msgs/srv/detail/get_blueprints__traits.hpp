// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:srv/GetBlueprints.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__TRAITS_HPP_
#define CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/srv/detail/get_blueprints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetBlueprints_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: filter
  {
    out << "filter: ";
    rosidl_generator_traits::value_to_yaml(msg.filter, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetBlueprints_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: filter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filter: ";
    rosidl_generator_traits::value_to_yaml(msg.filter, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBlueprints_Request & msg, bool use_flow_style = false)
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

}  // namespace carla_msgs

namespace rosidl_generator_traits
{

[[deprecated("use carla_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_msgs::srv::GetBlueprints_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::GetBlueprints_Request & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::GetBlueprints_Request>()
{
  return "carla_msgs::srv::GetBlueprints_Request";
}

template<>
inline const char * name<carla_msgs::srv::GetBlueprints_Request>()
{
  return "carla_msgs/srv/GetBlueprints_Request";
}

template<>
struct has_fixed_size<carla_msgs::srv::GetBlueprints_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::srv::GetBlueprints_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::srv::GetBlueprints_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetBlueprints_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: blueprints
  {
    if (msg.blueprints.size() == 0) {
      out << "blueprints: []";
    } else {
      out << "blueprints: [";
      size_t pending_items = msg.blueprints.size();
      for (auto item : msg.blueprints) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const GetBlueprints_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: blueprints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.blueprints.size() == 0) {
      out << "blueprints: []\n";
    } else {
      out << "blueprints:\n";
      for (auto item : msg.blueprints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetBlueprints_Response & msg, bool use_flow_style = false)
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

}  // namespace carla_msgs

namespace rosidl_generator_traits
{

[[deprecated("use carla_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const carla_msgs::srv::GetBlueprints_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::GetBlueprints_Response & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::GetBlueprints_Response>()
{
  return "carla_msgs::srv::GetBlueprints_Response";
}

template<>
inline const char * name<carla_msgs::srv::GetBlueprints_Response>()
{
  return "carla_msgs/srv/GetBlueprints_Response";
}

template<>
struct has_fixed_size<carla_msgs::srv::GetBlueprints_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::srv::GetBlueprints_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::srv::GetBlueprints_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_msgs::srv::GetBlueprints>()
{
  return "carla_msgs::srv::GetBlueprints";
}

template<>
inline const char * name<carla_msgs::srv::GetBlueprints>()
{
  return "carla_msgs/srv/GetBlueprints";
}

template<>
struct has_fixed_size<carla_msgs::srv::GetBlueprints>
  : std::integral_constant<
    bool,
    has_fixed_size<carla_msgs::srv::GetBlueprints_Request>::value &&
    has_fixed_size<carla_msgs::srv::GetBlueprints_Response>::value
  >
{
};

template<>
struct has_bounded_size<carla_msgs::srv::GetBlueprints>
  : std::integral_constant<
    bool,
    has_bounded_size<carla_msgs::srv::GetBlueprints_Request>::value &&
    has_bounded_size<carla_msgs::srv::GetBlueprints_Response>::value
  >
{
};

template<>
struct is_service<carla_msgs::srv::GetBlueprints>
  : std::true_type
{
};

template<>
struct is_service_request<carla_msgs::srv::GetBlueprints_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carla_msgs::srv::GetBlueprints_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__SRV__DETAIL__GET_BLUEPRINTS__TRAITS_HPP_
