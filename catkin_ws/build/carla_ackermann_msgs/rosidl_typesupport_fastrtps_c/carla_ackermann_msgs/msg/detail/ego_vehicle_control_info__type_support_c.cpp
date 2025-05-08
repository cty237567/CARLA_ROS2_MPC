// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlInfo.idl
// generated code does not contain a copyright notice
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "carla_ackermann_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_info__struct.h"
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_info__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_current__functions.h"  // current
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_maxima__functions.h"  // restrictions
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_status__functions.h"  // status
#include "carla_ackermann_msgs/msg/detail/ego_vehicle_control_target__functions.h"  // target
#include "carla_msgs/msg/detail/carla_ego_vehicle_control__functions.h"  // output
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlCurrent(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlCurrent(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlCurrent)();
size_t get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlMaxima(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlMaxima(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlMaxima)();
size_t get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlStatus)();
size_t get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlTarget(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlTarget(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlTarget)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
size_t get_serialized_size_carla_msgs__msg__CarlaEgoVehicleControl(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
size_t max_serialized_size_carla_msgs__msg__CarlaEgoVehicleControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaEgoVehicleControl)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_carla_ackermann_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _EgoVehicleControlInfo__ros_msg_type = carla_ackermann_msgs__msg__EgoVehicleControlInfo;

static bool _EgoVehicleControlInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EgoVehicleControlInfo__ros_msg_type * ros_message = static_cast<const _EgoVehicleControlInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: restrictions
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlMaxima
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->restrictions, cdr))
    {
      return false;
    }
  }

  // Field name: target
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlTarget
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->target, cdr))
    {
      return false;
    }
  }

  // Field name: current
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlCurrent
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->current, cdr))
    {
      return false;
    }
  }

  // Field name: status
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlStatus
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->status, cdr))
    {
      return false;
    }
  }

  // Field name: output
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaEgoVehicleControl
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->output, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _EgoVehicleControlInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EgoVehicleControlInfo__ros_msg_type * ros_message = static_cast<_EgoVehicleControlInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: restrictions
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlMaxima
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->restrictions))
    {
      return false;
    }
  }

  // Field name: target
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlTarget
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->target))
    {
      return false;
    }
  }

  // Field name: current
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlCurrent
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->current))
    {
      return false;
    }
  }

  // Field name: status
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlStatus
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->status))
    {
      return false;
    }
  }

  // Field name: output
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaEgoVehicleControl
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->output))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_ackermann_msgs
size_t get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EgoVehicleControlInfo__ros_msg_type * ros_message = static_cast<const _EgoVehicleControlInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name restrictions

  current_alignment += get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlMaxima(
    &(ros_message->restrictions), current_alignment);
  // field.name target

  current_alignment += get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlTarget(
    &(ros_message->target), current_alignment);
  // field.name current

  current_alignment += get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlCurrent(
    &(ros_message->current), current_alignment);
  // field.name status

  current_alignment += get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlStatus(
    &(ros_message->status), current_alignment);
  // field.name output

  current_alignment += get_serialized_size_carla_msgs__msg__CarlaEgoVehicleControl(
    &(ros_message->output), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _EgoVehicleControlInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlInfo(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_ackermann_msgs
size_t max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlInfo(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: restrictions
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlMaxima(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: target
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlTarget(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: current
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlCurrent(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: status
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlStatus(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: output
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_carla_msgs__msg__CarlaEgoVehicleControl(
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
    using DataType = carla_ackermann_msgs__msg__EgoVehicleControlInfo;
    is_plain =
      (
      offsetof(DataType, output) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _EgoVehicleControlInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_carla_ackermann_msgs__msg__EgoVehicleControlInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EgoVehicleControlInfo = {
  "carla_ackermann_msgs::msg",
  "EgoVehicleControlInfo",
  _EgoVehicleControlInfo__cdr_serialize,
  _EgoVehicleControlInfo__cdr_deserialize,
  _EgoVehicleControlInfo__get_serialized_size,
  _EgoVehicleControlInfo__max_serialized_size
};

static rosidl_message_type_support_t _EgoVehicleControlInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EgoVehicleControlInfo,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_ackermann_msgs, msg, EgoVehicleControlInfo)() {
  return &_EgoVehicleControlInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
