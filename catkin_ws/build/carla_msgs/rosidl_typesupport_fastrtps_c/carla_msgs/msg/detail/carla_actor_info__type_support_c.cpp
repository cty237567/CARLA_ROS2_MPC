// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from carla_msgs:msg/CarlaActorInfo.idl
// generated code does not contain a copyright notice
#include "carla_msgs/msg/detail/carla_actor_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "carla_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "carla_msgs/msg/detail/carla_actor_info__struct.h"
#include "carla_msgs/msg/detail/carla_actor_info__functions.h"
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

#include "rosidl_runtime_c/string.h"  // rolename, type
#include "rosidl_runtime_c/string_functions.h"  // rolename, type

// forward declare type support functions


using _CarlaActorInfo__ros_msg_type = carla_msgs__msg__CarlaActorInfo;

static bool _CarlaActorInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CarlaActorInfo__ros_msg_type * ros_message = static_cast<const _CarlaActorInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: parent_id
  {
    cdr << ros_message->parent_id;
  }

  // Field name: type
  {
    const rosidl_runtime_c__String * str = &ros_message->type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: rolename
  {
    const rosidl_runtime_c__String * str = &ros_message->rolename;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _CarlaActorInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CarlaActorInfo__ros_msg_type * ros_message = static_cast<_CarlaActorInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: parent_id
  {
    cdr >> ros_message->parent_id;
  }

  // Field name: type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->type.data) {
      rosidl_runtime_c__String__init(&ros_message->type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'type'\n");
      return false;
    }
  }

  // Field name: rolename
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->rolename.data) {
      rosidl_runtime_c__String__init(&ros_message->rolename);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->rolename,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'rolename'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t get_serialized_size_carla_msgs__msg__CarlaActorInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CarlaActorInfo__ros_msg_type * ros_message = static_cast<const _CarlaActorInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name parent_id
  {
    size_t item_size = sizeof(ros_message->parent_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->type.size + 1);
  // field.name rolename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->rolename.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _CarlaActorInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_carla_msgs__msg__CarlaActorInfo(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_carla_msgs
size_t max_serialized_size_carla_msgs__msg__CarlaActorInfo(
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

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: parent_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: rolename
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = carla_msgs__msg__CarlaActorInfo;
    is_plain =
      (
      offsetof(DataType, rolename) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CarlaActorInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_carla_msgs__msg__CarlaActorInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CarlaActorInfo = {
  "carla_msgs::msg",
  "CarlaActorInfo",
  _CarlaActorInfo__cdr_serialize,
  _CarlaActorInfo__cdr_deserialize,
  _CarlaActorInfo__get_serialized_size,
  _CarlaActorInfo__max_serialized_size
};

static rosidl_message_type_support_t _CarlaActorInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CarlaActorInfo,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, carla_msgs, msg, CarlaActorInfo)() {
  return &_CarlaActorInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
