// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:srv/DestroyObject.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__SRV__DETAIL__DESTROY_OBJECT__TRAITS_HPP_
#define CARLA_MSGS__SRV__DETAIL__DESTROY_OBJECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/srv/detail/destroy_object__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DestroyObject_Request & msg,
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
  const DestroyObject_Request & msg,
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

inline std::string to_yaml(const DestroyObject_Request & msg, bool use_flow_style = false)
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
  const carla_msgs::srv::DestroyObject_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::DestroyObject_Request & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::DestroyObject_Request>()
{
  return "carla_msgs::srv::DestroyObject_Request";
}

template<>
inline const char * name<carla_msgs::srv::DestroyObject_Request>()
{
  return "carla_msgs/srv/DestroyObject_Request";
}

template<>
struct has_fixed_size<carla_msgs::srv::DestroyObject_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_msgs::srv::DestroyObject_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_msgs::srv::DestroyObject_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const DestroyObject_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DestroyObject_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DestroyObject_Response & msg, bool use_flow_style = false)
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
  const carla_msgs::srv::DestroyObject_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::DestroyObject_Response & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::DestroyObject_Response>()
{
  return "carla_msgs::srv::DestroyObject_Response";
}

template<>
inline const char * name<carla_msgs::srv::DestroyObject_Response>()
{
  return "carla_msgs/srv/DestroyObject_Response";
}

template<>
struct has_fixed_size<carla_msgs::srv::DestroyObject_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_msgs::srv::DestroyObject_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_msgs::srv::DestroyObject_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_msgs::srv::DestroyObject>()
{
  return "carla_msgs::srv::DestroyObject";
}

template<>
inline const char * name<carla_msgs::srv::DestroyObject>()
{
  return "carla_msgs/srv/DestroyObject";
}

template<>
struct has_fixed_size<carla_msgs::srv::DestroyObject>
  : std::integral_constant<
    bool,
    has_fixed_size<carla_msgs::srv::DestroyObject_Request>::value &&
    has_fixed_size<carla_msgs::srv::DestroyObject_Response>::value
  >
{
};

template<>
struct has_bounded_size<carla_msgs::srv::DestroyObject>
  : std::integral_constant<
    bool,
    has_bounded_size<carla_msgs::srv::DestroyObject_Request>::value &&
    has_bounded_size<carla_msgs::srv::DestroyObject_Response>::value
  >
{
};

template<>
struct is_service<carla_msgs::srv::DestroyObject>
  : std::true_type
{
};

template<>
struct is_service_request<carla_msgs::srv::DestroyObject_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carla_msgs::srv::DestroyObject_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__SRV__DETAIL__DESTROY_OBJECT__TRAITS_HPP_
