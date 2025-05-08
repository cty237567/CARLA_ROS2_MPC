// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from carla_ackermann_msgs:msg/EgoVehicleControlStatus.idl
// generated code does not contain a copyright notice

#ifndef CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__STRUCT_H_
#define CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EgoVehicleControlStatus in the package carla_ackermann_msgs.
/**
  * Copyright (c) 2018-2019 Intel Corporation.
  *
  * This work is licensed under the terms of the MIT license.
  * For a copy, see <https://opensource.org/licenses/MIT>.
  *
  * This represents the control status variables of the ego vehicle control
 */
typedef struct carla_ackermann_msgs__msg__EgoVehicleControlStatus
{
  /// the current control status
  rosidl_runtime_c__String status;
  /// speed controller
  uint8_t speed_control_activation_count;
  float speed_control_accel_delta;
  float speed_control_accel_target;
  /// acceleration controller
  float accel_control_pedal_delta;
  float accel_control_pedal_target;
  /// borders for lay off pedal
  float brake_upper_border;
  float throttle_lower_border;
} carla_ackermann_msgs__msg__EgoVehicleControlStatus;

// Struct for a sequence of carla_ackermann_msgs__msg__EgoVehicleControlStatus.
typedef struct carla_ackermann_msgs__msg__EgoVehicleControlStatus__Sequence
{
  carla_ackermann_msgs__msg__EgoVehicleControlStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_ackermann_msgs__msg__EgoVehicleControlStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARLA_ACKERMANN_MSGS__MSG__DETAIL__EGO_VEHICLE_CONTROL_STATUS__STRUCT_H_
