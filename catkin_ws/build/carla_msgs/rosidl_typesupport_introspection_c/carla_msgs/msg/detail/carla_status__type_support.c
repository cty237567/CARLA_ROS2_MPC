// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_msgs:msg/CarlaStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_msgs/msg/detail/carla_status__rosidl_typesupport_introspection_c.h"
#include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_msgs/msg/detail/carla_status__functions.h"
#include "carla_msgs/msg/detail/carla_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__msg__CarlaStatus__init(message_memory);
}

void carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_fini_function(void * message_memory)
{
  carla_msgs__msg__CarlaStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_member_array[4] = {
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaStatus, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fixed_delta_seconds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaStatus, fixed_delta_seconds),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "synchronous_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaStatus, synchronous_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "synchronous_mode_running",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaStatus, synchronous_mode_running),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_members = {
  "carla_msgs__msg",  // message namespace
  "CarlaStatus",  // message name
  4,  // number of fields
  sizeof(carla_msgs__msg__CarlaStatus),
  carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_member_array,  // message members
  carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_type_support_handle = {
  0,
  &carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, msg, CarlaStatus)() {
  if (!carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_type_support_handle.typesupport_identifier) {
    carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__msg__CarlaStatus__rosidl_typesupport_introspection_c__CarlaStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
