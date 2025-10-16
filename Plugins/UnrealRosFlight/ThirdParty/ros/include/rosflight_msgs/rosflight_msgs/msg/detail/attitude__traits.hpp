// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/msg/detail/attitude__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosflight_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Attitude & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: attitude
  {
    out << "attitude: ";
    to_flow_style_yaml(msg.attitude, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Attitude & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: attitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attitude:\n";
    to_block_style_yaml(msg.attitude, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Attitude & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rosflight_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosflight_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosflight_msgs::msg::Attitude & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::msg::Attitude & msg)
{
  return rosflight_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::msg::Attitude>()
{
  return "rosflight_msgs::msg::Attitude";
}

template<>
inline const char * name<rosflight_msgs::msg::Attitude>()
{
  return "rosflight_msgs/msg/Attitude";
}

template<>
struct has_fixed_size<rosflight_msgs::msg::Attitude>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rosflight_msgs::msg::Attitude>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rosflight_msgs::msg::Attitude>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_
