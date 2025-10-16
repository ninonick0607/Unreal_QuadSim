// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_

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
# define DEPRECATED__rosflight_msgs__msg__Command __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__msg__Command __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Command_
{
  using Type = Command_<ContainerAllocator>;

  explicit Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->ignore = 0;
      this->qx = 0.0f;
      this->qy = 0.0f;
      this->qz = 0.0f;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->fz = 0.0f;
    }
  }

  explicit Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->ignore = 0;
      this->qx = 0.0f;
      this->qy = 0.0f;
      this->qz = 0.0f;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->fz = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _ignore_type =
    uint8_t;
  _ignore_type ignore;
  using _qx_type =
    float;
  _qx_type qx;
  using _qy_type =
    float;
  _qy_type qy;
  using _qz_type =
    float;
  _qz_type qz;
  using _fx_type =
    float;
  _fx_type fx;
  using _fy_type =
    float;
  _fy_type fy;
  using _fz_type =
    float;
  _fz_type fz;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__ignore(
    const uint8_t & _arg)
  {
    this->ignore = _arg;
    return *this;
  }
  Type & set__qx(
    const float & _arg)
  {
    this->qx = _arg;
    return *this;
  }
  Type & set__qy(
    const float & _arg)
  {
    this->qy = _arg;
    return *this;
  }
  Type & set__qz(
    const float & _arg)
  {
    this->qz = _arg;
    return *this;
  }
  Type & set__fx(
    const float & _arg)
  {
    this->fx = _arg;
    return *this;
  }
  Type & set__fy(
    const float & _arg)
  {
    this->fy = _arg;
    return *this;
  }
  Type & set__fz(
    const float & _arg)
  {
    this->fz = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MODE_PASS_THROUGH =
    0u;
  static constexpr uint8_t MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE =
    1u;
  static constexpr uint8_t MODE_ROLL_PITCH_YAWRATE_THROTTLE =
    2u;
  static constexpr uint8_t IGNORE_NONE =
    0u;
  static constexpr uint8_t IGNORE_QX =
    1u;
  static constexpr uint8_t IGNORE_QY =
    2u;
  static constexpr uint8_t IGNORE_QZ =
    4u;
  static constexpr uint8_t IGNORE_FX =
    8u;
  static constexpr uint8_t IGNORE_FY =
    16u;
  static constexpr uint8_t IGNORE_FZ =
    32u;

  // pointer types
  using RawPtr =
    rosflight_msgs::msg::Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::msg::Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::msg::Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::msg::Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::msg::Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::msg::Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__msg__Command
    std::shared_ptr<rosflight_msgs::msg::Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__msg__Command
    std::shared_ptr<rosflight_msgs::msg::Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->ignore != other.ignore) {
      return false;
    }
    if (this->qx != other.qx) {
      return false;
    }
    if (this->qy != other.qy) {
      return false;
    }
    if (this->qz != other.qz) {
      return false;
    }
    if (this->fx != other.fx) {
      return false;
    }
    if (this->fy != other.fy) {
      return false;
    }
    if (this->fz != other.fz) {
      return false;
    }
    return true;
  }
  bool operator!=(const Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_

// alias to use template instance with default allocator
using Command =
  rosflight_msgs::msg::Command_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::MODE_PASS_THROUGH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_QX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_QY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_QZ;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_FX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_FY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Command_<ContainerAllocator>::IGNORE_FZ;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_HPP_
