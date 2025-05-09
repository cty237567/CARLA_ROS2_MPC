// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from carla_msgs:msg/CarlaTrafficLightInfoList.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_INFO_LIST__STRUCT_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_INFO_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'traffic_lights'
#include "carla_msgs/msg/detail/carla_traffic_light_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__carla_msgs__msg__CarlaTrafficLightInfoList __attribute__((deprecated))
#else
# define DEPRECATED__carla_msgs__msg__CarlaTrafficLightInfoList __declspec(deprecated)
#endif

namespace carla_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarlaTrafficLightInfoList_
{
  using Type = CarlaTrafficLightInfoList_<ContainerAllocator>;

  explicit CarlaTrafficLightInfoList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit CarlaTrafficLightInfoList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _traffic_lights_type =
    std::vector<carla_msgs::msg::CarlaTrafficLightInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::msg::CarlaTrafficLightInfo_<ContainerAllocator>>>;
  _traffic_lights_type traffic_lights;

  // setters for named parameter idiom
  Type & set__traffic_lights(
    const std::vector<carla_msgs::msg::CarlaTrafficLightInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<carla_msgs::msg::CarlaTrafficLightInfo_<ContainerAllocator>>> & _arg)
  {
    this->traffic_lights = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> *;
  using ConstRawPtr =
    const carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__carla_msgs__msg__CarlaTrafficLightInfoList
    std::shared_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__carla_msgs__msg__CarlaTrafficLightInfoList
    std::shared_ptr<carla_msgs::msg::CarlaTrafficLightInfoList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarlaTrafficLightInfoList_ & other) const
  {
    if (this->traffic_lights != other.traffic_lights) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarlaTrafficLightInfoList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarlaTrafficLightInfoList_

// alias to use template instance with default allocator
using CarlaTrafficLightInfoList =
  carla_msgs::msg::CarlaTrafficLightInfoList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace carla_msgs

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_TRAFFIC_LIGHT_INFO_LIST__STRUCT_HPP_
