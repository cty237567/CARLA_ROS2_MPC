// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:srv/SpawnObject.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__SRV__DETAIL__SPAWN_OBJECT__TRAITS_HPP_
#define CARLA_MSGS__SRV__DETAIL__SPAWN_OBJECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/srv/detail/spawn_object__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'attributes'
#include "diagnostic_msgs/msg/detail/key_value__traits.hpp"
// Member 'transform'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SpawnObject_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: attributes
  {
    if (msg.attributes.size() == 0) {
      out << "attributes: []";
    } else {
      out << "attributes: [";
      size_t pending_items = msg.attributes.size();
      for (auto item : msg.attributes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: transform
  {
    out << "transform: ";
    to_flow_style_yaml(msg.transform, out);
    out << ", ";
  }

  // member: attach_to
  {
    out << "attach_to: ";
    rosidl_generator_traits::value_to_yaml(msg.attach_to, out);
    out << ", ";
  }

  // member: random_pose
  {
    out << "random_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.random_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpawnObject_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: attributes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.attributes.size() == 0) {
      out << "attributes: []\n";
    } else {
      out << "attributes:\n";
      for (auto item : msg.attributes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: transform
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transform:\n";
    to_block_style_yaml(msg.transform, out, indentation + 2);
  }

  // member: attach_to
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attach_to: ";
    rosidl_generator_traits::value_to_yaml(msg.attach_to, out);
    out << "\n";
  }

  // member: random_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "random_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.random_pose, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpawnObject_Request & msg, bool use_flow_style = false)
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
  const carla_msgs::srv::SpawnObject_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::SpawnObject_Request & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::SpawnObject_Request>()
{
  return "carla_msgs::srv::SpawnObject_Request";
}

template<>
inline const char * name<carla_msgs::srv::SpawnObject_Request>()
{
  return "carla_msgs/srv/SpawnObject_Request";
}

template<>
struct has_fixed_size<carla_msgs::srv::SpawnObject_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::srv::SpawnObject_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::srv::SpawnObject_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace carla_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SpawnObject_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: error_string
  {
    out << "error_string: ";
    rosidl_generator_traits::value_to_yaml(msg.error_string, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpawnObject_Response & msg,
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

  // member: error_string
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_string: ";
    rosidl_generator_traits::value_to_yaml(msg.error_string, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpawnObject_Response & msg, bool use_flow_style = false)
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
  const carla_msgs::srv::SpawnObject_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::srv::SpawnObject_Response & msg)
{
  return carla_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::srv::SpawnObject_Response>()
{
  return "carla_msgs::srv::SpawnObject_Response";
}

template<>
inline const char * name<carla_msgs::srv::SpawnObject_Response>()
{
  return "carla_msgs/srv/SpawnObject_Response";
}

template<>
struct has_fixed_size<carla_msgs::srv::SpawnObject_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::srv::SpawnObject_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::srv::SpawnObject_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_msgs::srv::SpawnObject>()
{
  return "carla_msgs::srv::SpawnObject";
}

template<>
inline const char * name<carla_msgs::srv::SpawnObject>()
{
  return "carla_msgs/srv/SpawnObject";
}

template<>
struct has_fixed_size<carla_msgs::srv::SpawnObject>
  : std::integral_constant<
    bool,
    has_fixed_size<carla_msgs::srv::SpawnObject_Request>::value &&
    has_fixed_size<carla_msgs::srv::SpawnObject_Response>::value
  >
{
};

template<>
struct has_bounded_size<carla_msgs::srv::SpawnObject>
  : std::integral_constant<
    bool,
    has_bounded_size<carla_msgs::srv::SpawnObject_Request>::value &&
    has_bounded_size<carla_msgs::srv::SpawnObject_Response>::value
  >
{
};

template<>
struct is_service<carla_msgs::srv::SpawnObject>
  : std::true_type
{
};

template<>
struct is_service_request<carla_msgs::srv::SpawnObject_Request>
  : std::true_type
{
};

template<>
struct is_service_response<carla_msgs::srv::SpawnObject_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__SRV__DETAIL__SPAWN_OBJECT__TRAITS_HPP_
