// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from carla_ros_scenario_runner_types:msg/CarlaScenarioList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario_list__rosidl_typesupport_introspection_c.h"
#include "carla_ros_scenario_runner_types/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario_list__functions.h"
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario_list__struct.h"


// Include directives for member types
// Member `scenarios`
#include "carla_ros_scenario_runner_types/msg/carla_scenario.h"
// Member `scenarios`
#include "carla_ros_scenario_runner_types/msg/detail/carla_scenario__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__init(message_memory);
}

void carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_fini_function(void * message_memory)
{
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__fini(message_memory);
}

size_t carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__size_function__CarlaScenarioList__scenarios(
  const void * untyped_member)
{
  const carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence * member =
    (const carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence *)(untyped_member);
  return member->size;
}

const void * carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_const_function__CarlaScenarioList__scenarios(
  const void * untyped_member, size_t index)
{
  const carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence * member =
    (const carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence *)(untyped_member);
  return &member->data[index];
}

void * carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_function__CarlaScenarioList__scenarios(
  void * untyped_member, size_t index)
{
  carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence * member =
    (carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence *)(untyped_member);
  return &member->data[index];
}

void carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__fetch_function__CarlaScenarioList__scenarios(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const carla_ros_scenario_runner_types__msg__CarlaScenario * item =
    ((const carla_ros_scenario_runner_types__msg__CarlaScenario *)
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_const_function__CarlaScenarioList__scenarios(untyped_member, index));
  carla_ros_scenario_runner_types__msg__CarlaScenario * value =
    (carla_ros_scenario_runner_types__msg__CarlaScenario *)(untyped_value);
  *value = *item;
}

void carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__assign_function__CarlaScenarioList__scenarios(
  void * untyped_member, size_t index, const void * untyped_value)
{
  carla_ros_scenario_runner_types__msg__CarlaScenario * item =
    ((carla_ros_scenario_runner_types__msg__CarlaScenario *)
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_function__CarlaScenarioList__scenarios(untyped_member, index));
  const carla_ros_scenario_runner_types__msg__CarlaScenario * value =
    (const carla_ros_scenario_runner_types__msg__CarlaScenario *)(untyped_value);
  *item = *value;
}

bool carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__resize_function__CarlaScenarioList__scenarios(
  void * untyped_member, size_t size)
{
  carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence * member =
    (carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence *)(untyped_member);
  carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence__fini(member);
  return carla_ros_scenario_runner_types__msg__CarlaScenario__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_member_array[1] = {
  {
    "scenarios",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_ros_scenario_runner_types__msg__CarlaScenarioList, scenarios),  // bytes offset in struct
    NULL,  // default value
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__size_function__CarlaScenarioList__scenarios,  // size() function pointer
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_const_function__CarlaScenarioList__scenarios,  // get_const(index) function pointer
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__get_function__CarlaScenarioList__scenarios,  // get(index) function pointer
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__fetch_function__CarlaScenarioList__scenarios,  // fetch(index, &value) function pointer
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__assign_function__CarlaScenarioList__scenarios,  // assign(index, value) function pointer
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__resize_function__CarlaScenarioList__scenarios  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_members = {
  "carla_ros_scenario_runner_types__msg",  // message namespace
  "CarlaScenarioList",  // message name
  1,  // number of fields
  sizeof(carla_ros_scenario_runner_types__msg__CarlaScenarioList),
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_member_array,  // message members
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_init_function,  // function to initialize message memory (memory has to be allocated)
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_type_support_handle = {
  0,
  &carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_carla_ros_scenario_runner_types
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_ros_scenario_runner_types, msg, CarlaScenarioList)() {
  carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, carla_ros_scenario_runner_types, msg, CarlaScenario)();
  if (!carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_type_support_handle.typesupport_identifier) {
    carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &carla_ros_scenario_runner_types__msg__CarlaScenarioList__rosidl_typesupport_introspection_c__CarlaScenarioList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
