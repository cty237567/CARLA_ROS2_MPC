// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_msgs:srv/GetBlueprints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_msgs/srv/detail/get_blueprints__rosidl_typesupport_introspection_c.h"
#include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_msgs/srv/detail/get_blueprints__functions.h"
#include "carla_msgs/srv/detail/get_blueprints__struct.h"


// Include directives for member types
// Member `filter`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__srv__GetBlueprints_Request__init(message_memory);
}

void carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_fini_function(void * message_memory)
{
  carla_msgs__srv__GetBlueprints_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_member_array[1] = {
  {
    "filter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__GetBlueprints_Request, filter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_members = {
  "carla_msgs__srv",  // message namespace
  "GetBlueprints_Request",  // message name
  1,  // number of fields
  sizeof(carla_msgs__srv__GetBlueprints_Request),
  carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_member_array,  // message members
  carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_type_support_handle = {
  0,
  &carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Request)() {
  if (!carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__srv__GetBlueprints_Request__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "carla_msgs/srv/detail/get_blueprints__rosidl_typesupport_introspection_c.h"
// already included above
// #include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "carla_msgs/srv/detail/get_blueprints__functions.h"
// already included above
// #include "carla_msgs/srv/detail/get_blueprints__struct.h"


// Include directives for member types
// Member `blueprints`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_msgs__srv__GetBlueprints_Response__init(message_memory);
}

void carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_fini_function(void * message_memory)
{
  carla_msgs__srv__GetBlueprints_Response__fini(message_memory);
}

size_t carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__size_function__GetBlueprints_Response__blueprints(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_const_function__GetBlueprints_Response__blueprints(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_function__GetBlueprints_Response__blueprints(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__fetch_function__GetBlueprints_Response__blueprints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_const_function__GetBlueprints_Response__blueprints(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__assign_function__GetBlueprints_Response__blueprints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_function__GetBlueprints_Response__blueprints(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__resize_function__GetBlueprints_Response__blueprints(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_member_array[1] = {
  {
    "blueprints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs__srv__GetBlueprints_Response, blueprints),  // bytes offset in struct
    NULL,  // default value
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__size_function__GetBlueprints_Response__blueprints,  // size() function pointer
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_const_function__GetBlueprints_Response__blueprints,  // get_const(index) function pointer
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__get_function__GetBlueprints_Response__blueprints,  // get(index) function pointer
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__fetch_function__GetBlueprints_Response__blueprints,  // fetch(index, &value) function pointer
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__assign_function__GetBlueprints_Response__blueprints,  // assign(index, value) function pointer
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__resize_function__GetBlueprints_Response__blueprints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_members = {
  "carla_msgs__srv",  // message namespace
  "GetBlueprints_Response",  // message name
  1,  // number of fields
  sizeof(carla_msgs__srv__GetBlueprints_Response),
  carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_member_array,  // message members
  carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_type_support_handle = {
  0,
  &carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Response)() {
  if (!carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_msgs__srv__GetBlueprints_Response__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "carla_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "carla_msgs/srv/detail/get_blueprints__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_members = {
  "carla_msgs__srv",  // service namespace
  "GetBlueprints",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_Request_message_type_support_handle,
  NULL  // response message
  // carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_Response_message_type_support_handle
};

static rosidl_service_type_support_t carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_type_support_handle = {
  0,
  &carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints)() {
  if (!carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_type_support_handle.typesupport_identifier) {
    carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_msgs, srv, GetBlueprints_Response)()->data;
  }

  return &carla_msgs__srv__detail__get_blueprints__rosidl_typesupport_introspection_c__GetBlueprints_service_type_support_handle;
}
