// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rosflight_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__msg__Status __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->failsafe = false;
      this->rc_override = false;
      this->offboard = false;
      this->control_mode = 0;
      this->error_code = 0;
      this->num_errors = 0;
      this->loop_time_us = 0;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->failsafe = false;
      this->rc_override = false;
      this->offboard = false;
      this->control_mode = 0;
      this->error_code = 0;
      this->num_errors = 0;
      this->loop_time_us = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _armed_type =
    bool;
  _armed_type armed;
  using _failsafe_type =
    bool;
  _failsafe_type failsafe;
  using _rc_override_type =
    bool;
  _rc_override_type rc_override;
  using _offboard_type =
    bool;
  _offboard_type offboard;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _error_code_type =
    uint8_t;
  _error_code_type error_code;
  using _num_errors_type =
    int16_t;
  _num_errors_type num_errors;
  using _loop_time_us_type =
    int16_t;
  _loop_time_us_type loop_time_us;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__failsafe(
    const bool & _arg)
  {
    this->failsafe = _arg;
    return *this;
  }
  Type & set__rc_override(
    const bool & _arg)
  {
    this->rc_override = _arg;
    return *this;
  }
  Type & set__offboard(
    const bool & _arg)
  {
    this->offboard = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint8_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__num_errors(
    const int16_t & _arg)
  {
    this->num_errors = _arg;
    return *this;
  }
  Type & set__loop_time_us(
    const int16_t & _arg)
  {
    this->loop_time_us = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosflight_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__msg__Status
    std::shared_ptr<rosflight_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__msg__Status
    std::shared_ptr<rosflight_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->armed != other.armed) {
      return false;
    }
    if (this->failsafe != other.failsafe) {
      return false;
    }
    if (this->rc_override != other.rc_override) {
      return false;
    }
    if (this->offboard != other.offboard) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->num_errors != other.num_errors) {
      return false;
    }
    if (this->loop_time_us != other.loop_time_us) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  rosflight_msgs::msg::Status_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
