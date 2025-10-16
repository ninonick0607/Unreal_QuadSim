// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__rosflight_msgs__msg__AuxCommand __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__msg__AuxCommand __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AuxCommand_
{
  using Type = AuxCommand_<ContainerAllocator>;

  explicit AuxCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 14>::iterator, uint8_t>(this->type_array.begin(), this->type_array.end(), 0);
      std::fill<typename std::array<float, 14>::iterator, float>(this->values.begin(), this->values.end(), 0.0f);
    }
  }

  explicit AuxCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    type_array(_alloc),
    values(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 14>::iterator, uint8_t>(this->type_array.begin(), this->type_array.end(), 0);
      std::fill<typename std::array<float, 14>::iterator, float>(this->values.begin(), this->values.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_array_type =
    std::array<uint8_t, 14>;
  _type_array_type type_array;
  using _values_type =
    std::array<float, 14>;
  _values_type values;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type_array(
    const std::array<uint8_t, 14> & _arg)
  {
    this->type_array = _arg;
    return *this;
  }
  Type & set__values(
    const std::array<float, 14> & _arg)
  {
    this->values = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t AUX_COMMAND_DISABLED =
    0u;
  static constexpr uint8_t AUX_COMMAND_SERVO =
    1u;
  static constexpr uint8_t AUX_COMMAND_MOTOR =
    2u;

  // pointer types
  using RawPtr =
    rosflight_msgs::msg::AuxCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::msg::AuxCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::AuxCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::AuxCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__msg__AuxCommand
    std::shared_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__msg__AuxCommand
    std::shared_ptr<rosflight_msgs::msg::AuxCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AuxCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type_array != other.type_array) {
      return false;
    }
    if (this->values != other.values) {
      return false;
    }
    return true;
  }
  bool operator!=(const AuxCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AuxCommand_

// alias to use template instance with default allocator
using AuxCommand =
  rosflight_msgs::msg::AuxCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AuxCommand_<ContainerAllocator>::AUX_COMMAND_DISABLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AuxCommand_<ContainerAllocator>::AUX_COMMAND_SERVO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t AuxCommand_<ContainerAllocator>::AUX_COMMAND_MOTOR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_
