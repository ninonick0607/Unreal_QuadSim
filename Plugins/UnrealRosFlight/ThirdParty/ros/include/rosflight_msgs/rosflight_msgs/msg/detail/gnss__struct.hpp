// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:msg/GNSS.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_HPP_

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
# define DEPRECATED__rosflight_msgs__msg__GNSS __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__msg__GNSS __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GNSS_
{
  using Type = GNSS_<ContainerAllocator>;

  explicit GNSS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->num_sat = 0;
      this->lat = 0.0;
      this->lon = 0.0;
      this->alt = 0.0f;
      this->horizontal_accuracy = 0.0f;
      this->vertical_accuracy = 0.0f;
      this->vel_n = 0.0f;
      this->vel_e = 0.0f;
      this->vel_d = 0.0f;
      this->speed_accuracy = 0.0f;
      this->gnss_unix_seconds = 0ll;
      this->gnss_unix_nanos = 0l;
    }
  }

  explicit GNSS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fix_type = 0;
      this->num_sat = 0;
      this->lat = 0.0;
      this->lon = 0.0;
      this->alt = 0.0f;
      this->horizontal_accuracy = 0.0f;
      this->vertical_accuracy = 0.0f;
      this->vel_n = 0.0f;
      this->vel_e = 0.0f;
      this->vel_d = 0.0f;
      this->speed_accuracy = 0.0f;
      this->gnss_unix_seconds = 0ll;
      this->gnss_unix_nanos = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _num_sat_type =
    uint8_t;
  _num_sat_type num_sat;
  using _lat_type =
    double;
  _lat_type lat;
  using _lon_type =
    double;
  _lon_type lon;
  using _alt_type =
    float;
  _alt_type alt;
  using _horizontal_accuracy_type =
    float;
  _horizontal_accuracy_type horizontal_accuracy;
  using _vertical_accuracy_type =
    float;
  _vertical_accuracy_type vertical_accuracy;
  using _vel_n_type =
    float;
  _vel_n_type vel_n;
  using _vel_e_type =
    float;
  _vel_e_type vel_e;
  using _vel_d_type =
    float;
  _vel_d_type vel_d;
  using _speed_accuracy_type =
    float;
  _speed_accuracy_type speed_accuracy;
  using _gnss_unix_seconds_type =
    int64_t;
  _gnss_unix_seconds_type gnss_unix_seconds;
  using _gnss_unix_nanos_type =
    int32_t;
  _gnss_unix_nanos_type gnss_unix_nanos;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fix_type(
    const uint8_t & _arg)
  {
    this->fix_type = _arg;
    return *this;
  }
  Type & set__num_sat(
    const uint8_t & _arg)
  {
    this->num_sat = _arg;
    return *this;
  }
  Type & set__lat(
    const double & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const double & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__alt(
    const float & _arg)
  {
    this->alt = _arg;
    return *this;
  }
  Type & set__horizontal_accuracy(
    const float & _arg)
  {
    this->horizontal_accuracy = _arg;
    return *this;
  }
  Type & set__vertical_accuracy(
    const float & _arg)
  {
    this->vertical_accuracy = _arg;
    return *this;
  }
  Type & set__vel_n(
    const float & _arg)
  {
    this->vel_n = _arg;
    return *this;
  }
  Type & set__vel_e(
    const float & _arg)
  {
    this->vel_e = _arg;
    return *this;
  }
  Type & set__vel_d(
    const float & _arg)
  {
    this->vel_d = _arg;
    return *this;
  }
  Type & set__speed_accuracy(
    const float & _arg)
  {
    this->speed_accuracy = _arg;
    return *this;
  }
  Type & set__gnss_unix_seconds(
    const int64_t & _arg)
  {
    this->gnss_unix_seconds = _arg;
    return *this;
  }
  Type & set__gnss_unix_nanos(
    const int32_t & _arg)
  {
    this->gnss_unix_nanos = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GNSS_FIX_TYPE_NO_FIX =
    0u;
  static constexpr uint8_t GNSS_FIX_TYPE_DEAD_RECKONING_ONLY =
    1u;
  static constexpr uint8_t GNSS_FIX_TYPE_2D_FIX =
    2u;
  static constexpr uint8_t GNSS_FIX_TYPE_3D_FIX =
    3u;
  static constexpr uint8_t GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING =
    4u;
  static constexpr uint8_t GNSS_FIX_TYPE_TIME_FIX_ONLY =
    5u;

  // pointer types
  using RawPtr =
    rosflight_msgs::msg::GNSS_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::msg::GNSS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::GNSS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::msg::GNSS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__msg__GNSS
    std::shared_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__msg__GNSS
    std::shared_ptr<rosflight_msgs::msg::GNSS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GNSS_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
      return false;
    }
    if (this->num_sat != other.num_sat) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->alt != other.alt) {
      return false;
    }
    if (this->horizontal_accuracy != other.horizontal_accuracy) {
      return false;
    }
    if (this->vertical_accuracy != other.vertical_accuracy) {
      return false;
    }
    if (this->vel_n != other.vel_n) {
      return false;
    }
    if (this->vel_e != other.vel_e) {
      return false;
    }
    if (this->vel_d != other.vel_d) {
      return false;
    }
    if (this->speed_accuracy != other.speed_accuracy) {
      return false;
    }
    if (this->gnss_unix_seconds != other.gnss_unix_seconds) {
      return false;
    }
    if (this->gnss_unix_nanos != other.gnss_unix_nanos) {
      return false;
    }
    return true;
  }
  bool operator!=(const GNSS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GNSS_

// alias to use template instance with default allocator
using GNSS =
  rosflight_msgs::msg::GNSS_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_NO_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_DEAD_RECKONING_ONLY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_2D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_3D_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GNSS_<ContainerAllocator>::GNSS_FIX_TYPE_TIME_FIX_ONLY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_HPP_
