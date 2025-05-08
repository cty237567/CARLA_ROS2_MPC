// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from carla_waypoint_types:msg/CarlaWaypoint.idl
// generated code does not contain a copyright notice
#include "carla_waypoint_types/msg/detail/carla_waypoint__rosidl_typesupport_fastrtps_cpp.hpp"
#include "carla_waypoint_types/msg/detail/carla_waypoint__struct.hpp"

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
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Pose &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Pose &);
size_t get_serialized_size(
  const geometry_msgs::msg::Pose &,
  size_t current_alignment);
size_t
max_serialized_size_Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace carla_waypoint_types
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_waypoint_types
cdr_serialize(
  const carla_waypoint_types::msg::CarlaWaypoint & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: road_id
  cdr << ros_message.road_id;
  // Member: section_id
  cdr << ros_message.section_id;
  // Member: lane_id
  cdr << ros_message.lane_id;
  // Member: is_junction
  cdr << (ros_message.is_junction ? true : false);
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_waypoint_types
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  carla_waypoint_types::msg::CarlaWaypoint & ros_message)
{
  // Member: road_id
  cdr >> ros_message.road_id;

  // Member: section_id
  cdr >> ros_message.section_id;

  // Member: lane_id
  cdr >> ros_message.lane_id;

  // Member: is_junction
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_junction = tmp ? true : false;
  }

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_waypoint_types
get_serialized_size(
  const carla_waypoint_types::msg::CarlaWaypoint & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: road_id
  {
    size_t item_size = sizeof(ros_message.road_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: section_id
  {
    size_t item_size = sizeof(ros_message.section_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lane_id
  {
    size_t item_size = sizeof(ros_message.lane_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_junction
  {
    size_t item_size = sizeof(ros_message.is_junction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_carla_waypoint_types
max_serialized_size_CarlaWaypoint(
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


  // Member: road_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: section_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: lane_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: is_junction
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose(
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
    using DataType = carla_waypoint_types::msg::CarlaWaypoint;
    is_plain =
      (
      offsetof(DataType, pose) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _CarlaWaypoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const carla_waypoint_types::msg::CarlaWaypoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CarlaWaypoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<carla_waypoint_types::msg::CarlaWaypoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CarlaWaypoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const carla_waypoint_types::msg::CarlaWaypoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CarlaWaypoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_CarlaWaypoint(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _CarlaWaypoint__callbacks = {
  "carla_waypoint_types::msg",
  "CarlaWaypoint",
  _CarlaWaypoint__cdr_serialize,
  _CarlaWaypoint__cdr_deserialize,
  _CarlaWaypoint__get_serialized_size,
  _CarlaWaypoint__max_serialized_size
};

static rosidl_message_type_support_t _CarlaWaypoint__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CarlaWaypoint__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace carla_waypoint_types

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_carla_waypoint_types
const rosidl_message_type_support_t *
get_message_type_support_handle<carla_waypoint_types::msg::CarlaWaypoint>()
{
  return &carla_waypoint_types::msg::typesupport_fastrtps_cpp::_CarlaWaypoint__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, carla_waypoint_types, msg, CarlaWaypoint)() {
  return &carla_waypoint_types::msg::typesupport_fastrtps_cpp::_CarlaWaypoint__handle;
}

#ifdef __cplusplus
}
#endif
