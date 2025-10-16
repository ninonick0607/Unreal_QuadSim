// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/msg/detail/airspeed__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosflight_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Airspeed & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: differential_pressure
  {
    out << "differential_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.differential_pressure, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Airspeed & msg,
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

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: differential_pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "differential_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.differential_pressure, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Airspeed & msg, bool use_flow_style = false)
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
  const rosflight_msgs::msg::Airspeed & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::msg::Airspeed & msg)
{
  return rosflight_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::msg::Airspeed>()
{
  return "rosflight_msgs::msg::Airspeed";
}

template<>
inline const char * name<rosflight_msgs::msg::Airspeed>()
{
  return "rosflight_msgs/msg/Airspeed";
}

template<>
struct has_fixed_size<rosflight_msgs::msg::Airspeed>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rosflight_msgs::msg::Airspeed>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rosflight_msgs::msg::Airspeed>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__TRAITS_HPP_
