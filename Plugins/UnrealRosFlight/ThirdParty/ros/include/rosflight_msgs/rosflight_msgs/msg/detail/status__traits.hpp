// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosflight_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: armed
  {
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << ", ";
  }

  // member: failsafe
  {
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
    out << ", ";
  }

  // member: rc_override
  {
    out << "rc_override: ";
    rosidl_generator_traits::value_to_yaml(msg.rc_override, out);
    out << ", ";
  }

  // member: offboard
  {
    out << "offboard: ";
    rosidl_generator_traits::value_to_yaml(msg.offboard, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: num_errors
  {
    out << "num_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_errors, out);
    out << ", ";
  }

  // member: loop_time_us
  {
    out << "loop_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_time_us, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: armed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << "\n";
  }

  // member: failsafe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
    out << "\n";
  }

  // member: rc_override
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rc_override: ";
    rosidl_generator_traits::value_to_yaml(msg.rc_override, out);
    out << "\n";
  }

  // member: offboard
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offboard: ";
    rosidl_generator_traits::value_to_yaml(msg.offboard, out);
    out << "\n";
  }

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
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

  // member: num_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_errors, out);
    out << "\n";
  }

  // member: loop_time_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_time_us: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_time_us, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const rosflight_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::msg::Status & msg)
{
  return rosflight_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::msg::Status>()
{
  return "rosflight_msgs::msg::Status";
}

template<>
inline const char * name<rosflight_msgs::msg::Status>()
{
  return "rosflight_msgs/msg/Status";
}

template<>
struct has_fixed_size<rosflight_msgs::msg::Status>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rosflight_msgs::msg::Status>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rosflight_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
