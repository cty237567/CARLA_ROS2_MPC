// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlInfo.idl
// generated code does not contain a copyright notice
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_info__rosidl_typesupport_fastrtps_cpp.hpp"
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_info__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace carla_ackermann_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlMaxima &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  carla_ackermann_msgs::msg::EgoVehicleControlMaxima &);
size_t get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlMaxima &,
  size_t current_alignment);
size_t
max_serialized_size_EgoVehicleControlMaxima(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace carla_ackermann_msgs

namespace carla_ackermann_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlTarget &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  carla_ackermann_msgs::msg::EgoVehicleControlTarget &);
size_t get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlTarget &,
  size_t current_alignment);
size_t
max_serialized_size_EgoVehicleControlTarget(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace carla_ackermann_msgs

namespace carla_ackermann_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlCurrent &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  carla_ackermann_msgs::msg::EgoVehicleControlCurrent &);
size_t get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlCurrent &,
  size_t current_alignment);
size_t
max_serialized_size_EgoVehicleControlCurrent(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace carla_ackermann_msgs

namespace carla_ackermann_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlStatus &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  carla_ackermann_msgs::msg::EgoVehicleControlStatus &);
size_t get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlStatus &,
  size_t current_alignment);
size_t
max_serialized_size_EgoVehicleControlStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace carla_ackermann_msgs

namespace carla_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const carla_msgs::msg::CarlaEgoVehicleControl &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  carla_msgs::msg::CarlaEgoVehicleControl &);
size_t get_serialized_size(
  const carla_msgs::msg::CarlaEgoVehicleControl &,
  size_t current_alignment);
size_t
max_serialized_size_CarlaEgoVehicleControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace carla_msgs


namespace carla_ackermann_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: restrictions
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.restrictions,
    cdr);
  // Member: target
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.target,
    cdr);
  // Member: current
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.current,
    cdr);
  // Member: status
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.status,
    cdr);
  // Member: output
  carla_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.output,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  carla_ackermann_msgs::msg::EgoVehicleControlInfo & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: restrictions
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.restrictions);

  // Member: target
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.target);

  // Member: current
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.current);

  // Member: status
  carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.status);

  // Member: output
  carla_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.output);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlInfo & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: restrictions

  current_alignment +=
    carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.restrictions, current_alignment);
  // Member: target

  current_alignment +=
    carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.target, current_alignment);
  // Member: current

  current_alignment +=
    carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.current, current_alignment);
  // Member: status

  current_alignment +=
    carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.status, current_alignment);
  // Member: output

  current_alignment +=
    carla_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.output, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
max_serialized_size_EgoVehicleControlInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: restrictions
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_EgoVehicleControlMaxima(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: target
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_EgoVehicleControlTarget(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: current
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_EgoVehicleControlCurrent(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: status
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_EgoVehicleControlStatus(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: output
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        carla_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_CarlaEgoVehicleControl(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_ackermann_msgs::msg::EgoVehicleControlInfo;
    is_plain =
      (
      offsetof(DataType, output) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EgoVehicleControlInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const carla_ackermann_msgs::msg::EgoVehicleControlInfo *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EgoVehicleControlInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<carla_ackermann_msgs::msg::EgoVehicleControlInfo *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EgoVehicleControlInfo__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const carla_ackermann_msgs::msg::EgoVehicleControlInfo *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EgoVehicleControlInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EgoVehicleControlInfo(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EgoVehicleControlInfo__callbacks = {
  "carla_ackermann_msgs::msg",
  "EgoVehicleControlInfo",
  _EgoVehicleControlInfo__cdr_serialize,
  _EgoVehicleControlInfo__cdr_deserialize,
  _EgoVehicleControlInfo__get_serialized_size,
  _EgoVehicleControlInfo__max_serialized_size
};

static rosidl_message_type_support_t _EgoVehicleControlInfo__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EgoVehicleControlInfo__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace carla_ackermann_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_carla_ackermann_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<carla_ackermann_msgs::msg::EgoVehicleControlInfo>()
{
  return &carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::_EgoVehicleControlInfo__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, carla_ackermann_msgs, msg, EgoVehicleControlInfo)() {
  return &carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::_EgoVehicleControlInfo__handle;
}

#ifdef __cplusplus
}
#endif
