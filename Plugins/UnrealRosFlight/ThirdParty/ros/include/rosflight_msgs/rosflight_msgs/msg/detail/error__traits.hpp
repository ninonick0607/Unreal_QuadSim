// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__TRAITS_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/msg/detail/error__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosflight_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Error & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: reset_count
  {
    out << "reset_count: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_count, out);
    out << ", ";
  }

  // member: rearm
  {
    out << "rearm: ";
    rosidl_generator_traits::value_to_yaml(msg.rearm, out);
    out << ", ";
  }

  // member: pc
  {
    out << "pc: ";
    rosidl_generator_traits::value_to_yaml(msg.pc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Error & msg,
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

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: reset_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_count: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_count, out);
    out << "\n";
  }

  // member: rearm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rearm: ";
    rosidl_generator_traits::value_to_yaml(msg.rearm, out);
    out << "\n";
  }

  // member: pc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pc: ";
    rosidl_generator_traits::value_to_yaml(msg.pc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Error & msg, bool use_flow_style = false)
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
  const rosflight_msgs::msg::Error & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::msg::Error & msg)
{
  return rosflight_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::msg::Error>()
{
  return "rosflight_msgs::msg::Error";
}

template<>
inline const char * name<rosflight_msgs::msg::Error>()
{
  return "rosflight_msgs/msg/Error";
}

template<>
struct has_fixed_size<rosflight_msgs::msg::Error>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosflight_msgs::msg::Error>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosflight_msgs::msg::Error>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__TRAITS_HPP_
