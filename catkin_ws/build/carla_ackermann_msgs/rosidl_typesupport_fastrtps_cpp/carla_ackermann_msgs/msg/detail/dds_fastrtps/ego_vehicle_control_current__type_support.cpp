// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlCurrent.idl
// generated code does not contain a copyright notice
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_current__rosidl_typesupport_fastrtps_cpp.hpp"
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_current__struct.hpp"

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

namespace carla_ackermann_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
cdr_serialize(
  const carla_ackermann_msgs::msg::EgoVehicleControlCurrent & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: time_sec
  cdr << ros_message.time_sec;
  // Member: speed
  cdr << ros_message.speed;
  // Member: speed_abs
  cdr << ros_message.speed_abs;
  // Member: accel
  cdr << ros_message.accel;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  carla_ackermann_msgs::msg::EgoVehicleControlCurrent & ros_message)
{
  // Member: time_sec
  cdr >> ros_message.time_sec;

  // Member: speed
  cdr >> ros_message.speed;

  // Member: speed_abs
  cdr >> ros_message.speed_abs;

  // Member: accel
  cdr >> ros_message.accel;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
get_serialized_size(
  const carla_ackermann_msgs::msg::EgoVehicleControlCurrent & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: time_sec
  {
    size_t item_size = sizeof(ros_message.time_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_abs
  {
    size_t item_size = sizeof(ros_message.speed_abs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: accel
  {
    size_t item_size = sizeof(ros_message.accel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_ackermann_msgs
max_serialized_size_EgoVehicleControlCurrent(
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


  // Member: time_sec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed_abs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: accel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_ackermann_msgs::msg::EgoVehicleControlCurrent;
    is_plain =
      (
      offsetof(DataType, accel) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EgoVehicleControlCurrent__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const carla_ackermann_msgs::msg::EgoVehicleControlCurrent *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EgoVehicleControlCurrent__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<carla_ackermann_msgs::msg::EgoVehicleControlCurrent *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EgoVehicleControlCurrent__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const carla_ackermann_msgs::msg::EgoVehicleControlCurrent *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EgoVehicleControlCurrent__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EgoVehicleControlCurrent(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EgoVehicleControlCurrent__callbacks = {
  "carla_ackermann_msgs::msg",
  "EgoVehicleControlCurrent",
  _EgoVehicleControlCurrent__cdr_serialize,
  _EgoVehicleControlCurrent__cdr_deserialize,
  _EgoVehicleControlCurrent__get_serialized_size,
  _EgoVehicleControlCurrent__max_serialized_size
};

static rosidl_message_type_support_t _EgoVehicleControlCurrent__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EgoVehicleControlCurrent__callbacks,
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
get_message_type_support_handle<carla_ackermann_msgs::msg::EgoVehicleControlCurrent>()
{
  return &carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::_EgoVehicleControlCurrent__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, carla_ackermann_msgs, msg, EgoVehicleControlCurrent)() {
  return &carla_ackermann_msgs::msg::typesupport_fastrtps_cpp::_EgoVehicleControlCurrent__handle;
}

#ifdef __cplusplus
}
#endif
