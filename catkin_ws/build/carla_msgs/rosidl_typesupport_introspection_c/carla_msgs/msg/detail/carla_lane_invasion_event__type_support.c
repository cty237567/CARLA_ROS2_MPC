// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_msgs:msg/CarlaLaneInvasionEvent.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_msgs/msg/detail/carla_lane_invasion_event__rosidl_typesupport_introspection_c.h"
#include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_msgs/msg/detail/carla_lane_invasion_event__functions.h"
#include "carla_msgs/msg/detail/carla_lane_invasion_event__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `crossed_lane_markings`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__msg__CarlaLaneInvasionEvent__init(message_memory);
}

void carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_fini_function(void * message_memory)
{
  carla_msgs__msg__CarlaLaneInvasionEvent__fini(message_memory);
}

size_t carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__size_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_const_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__fetch_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_const_function__CarlaLaneInvasionEvent__crossed_lane_markings(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__assign_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_function__CarlaLaneInvasionEvent__crossed_lane_markings(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__resize_function__CarlaLaneInvasionEvent__crossed_lane_markings(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaLaneInvasionEvent, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "crossed_lane_markings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__msg__CarlaLaneInvasionEvent, crossed_lane_markings),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__size_function__CarlaLaneInvasionEvent__crossed_lane_markings,  // size() function pointer
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_const_function__CarlaLaneInvasionEvent__crossed_lane_markings,  // get_const(index) function pointer
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__get_function__CarlaLaneInvasionEvent__crossed_lane_markings,  // get(index) function pointer
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__fetch_function__CarlaLaneInvasionEvent__crossed_lane_markings,  // fetch(index, &value) function pointer
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__assign_function__CarlaLaneInvasionEvent__crossed_lane_markings,  // assign(index, value) function pointer
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__resize_function__CarlaLaneInvasionEvent__crossed_lane_markings  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_members = {
  "carla_msgs__msg",  // message namespace
  "CarlaLaneInvasionEvent",  // message name
  2,  // number of fields
  sizeof(carla_msgs__msg__CarlaLaneInvasionEvent),
  carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_member_array,  // message members
  carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_type_support_handle = {
  0,
  &carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, msg, CarlaLaneInvasionEvent)() {
  carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_type_support_handle.typesupport_identifier) {
    carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__msg__CarlaLaneInvasionEvent__rosidl_typesupport_introspection_c__CarlaLaneInvasionEvent_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
