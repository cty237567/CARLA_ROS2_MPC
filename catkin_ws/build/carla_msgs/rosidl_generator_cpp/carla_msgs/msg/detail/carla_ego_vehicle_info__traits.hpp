// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaEgoVehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_ego_vehicle_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'wheels'
#include "carla_msgs/msg/detail/carla_ego_vehicle_info_wheel__traits.hpp"
// Member 'center_of_mass'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaEgoVehicleInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: rolename
  {
    out << "rolename: ";
    rosidl_generator_traits::value_to_yaml(msg.rolename, out);
    out << ", ";
  }

  // member: wheels
  {
    if (msg.wheels.size() == 0) {
      out << "wheels: []";
    } else {
      out << "wheels: [";
      size_t pending_items = msg.wheels.size();
      for (auto item : msg.wheels) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_rpm
  {
    out << "max_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.max_rpm, out);
    out << ", ";
  }

  // member: moi
  {
    out << "moi: ";
    rosidl_generator_traits::value_to_yaml(msg.moi, out);
    out << ", ";
  }

  // member: damping_rate_full_throttle
  {
    out << "damping_rate_full_throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_full_throttle, out);
    out << ", ";
  }

  // member: damping_rate_zero_throttle_clutch_engaged
  {
    out << "damping_rate_zero_throttle_clutch_engaged: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_zero_throttle_clutch_engaged, out);
    out << ", ";
  }

  // member: damping_rate_zero_throttle_clutch_disengaged
  {
    out << "damping_rate_zero_throttle_clutch_disengaged: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_zero_throttle_clutch_disengaged, out);
    out << ", ";
  }

  // member: use_gear_autobox
  {
    out << "use_gear_autobox: ";
    rosidl_generator_traits::value_to_yaml(msg.use_gear_autobox, out);
    out << ", ";
  }

  // member: gear_switch_time
  {
    out << "gear_switch_time: ";
    rosidl_generator_traits::value_to_yaml(msg.gear_switch_time, out);
    out << ", ";
  }

  // member: clutch_strength
  {
    out << "clutch_strength: ";
    rosidl_generator_traits::value_to_yaml(msg.clutch_strength, out);
    out << ", ";
  }

  // member: mass
  {
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << ", ";
  }

  // member: drag_coefficient
  {
    out << "drag_coefficient: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_coefficient, out);
    out << ", ";
  }

  // member: center_of_mass
  {
    out << "center_of_mass: ";
    to_flow_style_yaml(msg.center_of_mass, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaEgoVehicleInfo & msg,
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

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: rolename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rolename: ";
    rosidl_generator_traits::value_to_yaml(msg.rolename, out);
    out << "\n";
  }

  // member: wheels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheels.size() == 0) {
      out << "wheels: []\n";
    } else {
      out << "wheels:\n";
      for (auto item : msg.wheels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: max_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.max_rpm, out);
    out << "\n";
  }

  // member: moi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moi: ";
    rosidl_generator_traits::value_to_yaml(msg.moi, out);
    out << "\n";
  }

  // member: damping_rate_full_throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_rate_full_throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_full_throttle, out);
    out << "\n";
  }

  // member: damping_rate_zero_throttle_clutch_engaged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_rate_zero_throttle_clutch_engaged: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_zero_throttle_clutch_engaged, out);
    out << "\n";
  }

  // member: damping_rate_zero_throttle_clutch_disengaged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "damping_rate_zero_throttle_clutch_disengaged: ";
    rosidl_generator_traits::value_to_yaml(msg.damping_rate_zero_throttle_clutch_disengaged, out);
    out << "\n";
  }

  // member: use_gear_autobox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_gear_autobox: ";
    rosidl_generator_traits::value_to_yaml(msg.use_gear_autobox, out);
    out << "\n";
  }

  // member: gear_switch_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gear_switch_time: ";
    rosidl_generator_traits::value_to_yaml(msg.gear_switch_time, out);
    out << "\n";
  }

  // member: clutch_strength
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clutch_strength: ";
    rosidl_generator_traits::value_to_yaml(msg.clutch_strength, out);
    out << "\n";
  }

  // member: mass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << "\n";
  }

  // member: drag_coefficient
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drag_coefficient: ";
    rosidl_generator_traits::value_to_yaml(msg.drag_coefficient, out);
    out << "\n";
  }

  // member: center_of_mass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_of_mass:\n";
    to_block_style_yaml(msg.center_of_mass, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaEgoVehicleInfo & msg, bool use_flow_style = false)
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
  const carla_msgs::msg::CarlaEgoVehicleInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaEgoVehicleInfo & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaEgoVehicleInfo>()
{
  return "carla_msgs::msg::CarlaEgoVehicleInfo";
}

template<>
inline const char * name<carla_msgs::msg::CarlaEgoVehicleInfo>()
{
  return "carla_msgs/msg/CarlaEgoVehicleInfo";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaEgoVehicleInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaEgoVehicleInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<carla_msgs::msg::CarlaEgoVehicleInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_EGO_VEHICLE_INFO__TRAITS_HPP_
