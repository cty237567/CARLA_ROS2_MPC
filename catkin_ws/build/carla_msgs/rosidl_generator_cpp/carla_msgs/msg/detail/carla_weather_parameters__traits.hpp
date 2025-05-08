// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaWeatherParameters.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_WEATHER_PARAMETERS__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_WEATHER_PARAMETERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "carla_msgs/msg/detail/carla_weather_parameters__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace carla_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CarlaWeatherParameters & msg,
  std::ostream & out)
{
  out << "{";
  // member: cloudiness
  {
    out << "cloudiness: ";
    rosidl_generator_traits::value_to_yaml(msg.cloudiness, out);
    out << ", ";
  }

  // member: precipitation
  {
    out << "precipitation: ";
    rosidl_generator_traits::value_to_yaml(msg.precipitation, out);
    out << ", ";
  }

  // member: precipitation_deposits
  {
    out << "precipitation_deposits: ";
    rosidl_generator_traits::value_to_yaml(msg.precipitation_deposits, out);
    out << ", ";
  }

  // member: wind_intensity
  {
    out << "wind_intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.wind_intensity, out);
    out << ", ";
  }

  // member: fog_density
  {
    out << "fog_density: ";
    rosidl_generator_traits::value_to_yaml(msg.fog_density, out);
    out << ", ";
  }

  // member: fog_distance
  {
    out << "fog_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.fog_distance, out);
    out << ", ";
  }

  // member: wetness
  {
    out << "wetness: ";
    rosidl_generator_traits::value_to_yaml(msg.wetness, out);
    out << ", ";
  }

  // member: sun_azimuth_angle
  {
    out << "sun_azimuth_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sun_azimuth_angle, out);
    out << ", ";
  }

  // member: sun_altitude_angle
  {
    out << "sun_altitude_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sun_altitude_angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CarlaWeatherParameters & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cloudiness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloudiness: ";
    rosidl_generator_traits::value_to_yaml(msg.cloudiness, out);
    out << "\n";
  }

  // member: precipitation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "precipitation: ";
    rosidl_generator_traits::value_to_yaml(msg.precipitation, out);
    out << "\n";
  }

  // member: precipitation_deposits
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "precipitation_deposits: ";
    rosidl_generator_traits::value_to_yaml(msg.precipitation_deposits, out);
    out << "\n";
  }

  // member: wind_intensity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wind_intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.wind_intensity, out);
    out << "\n";
  }

  // member: fog_density
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fog_density: ";
    rosidl_generator_traits::value_to_yaml(msg.fog_density, out);
    out << "\n";
  }

  // member: fog_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fog_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.fog_distance, out);
    out << "\n";
  }

  // member: wetness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wetness: ";
    rosidl_generator_traits::value_to_yaml(msg.wetness, out);
    out << "\n";
  }

  // member: sun_azimuth_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sun_azimuth_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sun_azimuth_angle, out);
    out << "\n";
  }

  // member: sun_altitude_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sun_altitude_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.sun_altitude_angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CarlaWeatherParameters & msg, bool use_flow_style = false)
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
  const carla_msgs::msg::CarlaWeatherParameters & msg,
  std::ostream & out, size_t indentation = 0)
{
  carla_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use carla_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const carla_msgs::msg::CarlaWeatherParameters & msg)
{
  return carla_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<carla_msgs::msg::CarlaWeatherParameters>()
{
  return "carla_msgs::msg::CarlaWeatherParameters";
}

template<>
inline const char * name<carla_msgs::msg::CarlaWeatherParameters>()
{
  return "carla_msgs/msg/CarlaWeatherParameters";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaWeatherParameters>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaWeatherParameters>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<carla_msgs::msg::CarlaWeatherParameters>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_WEATHER_PARAMETERS__TRAITS_HPP_
