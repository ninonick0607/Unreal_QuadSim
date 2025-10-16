// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:msg/GNSS.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__TRAITS_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosflight_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GNSS & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: fix_type
  {
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << ", ";
  }

  // member: num_sat
  {
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
    out << ", ";
  }

  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << ", ";
  }

  // member: alt
  {
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
    out << ", ";
  }

  // member: horizontal_accuracy
  {
    out << "horizontal_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.horizontal_accuracy, out);
    out << ", ";
  }

  // member: vertical_accuracy
  {
    out << "vertical_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.vertical_accuracy, out);
    out << ", ";
  }

  // member: vel_n
  {
    out << "vel_n: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_n, out);
    out << ", ";
  }

  // member: vel_e
  {
    out << "vel_e: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_e, out);
    out << ", ";
  }

  // member: vel_d
  {
    out << "vel_d: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_d, out);
    out << ", ";
  }

  // member: speed_accuracy
  {
    out << "speed_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_accuracy, out);
    out << ", ";
  }

  // member: gnss_unix_seconds
  {
    out << "gnss_unix_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_unix_seconds, out);
    out << ", ";
  }

  // member: gnss_unix_nanos
  {
    out << "gnss_unix_nanos: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_unix_nanos, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GNSS & msg,
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

  // member: fix_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_type: ";
    rosidl_generator_traits::value_to_yaml(msg.fix_type, out);
    out << "\n";
  }

  // member: num_sat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sat: ";
    rosidl_generator_traits::value_to_yaml(msg.num_sat, out);
    out << "\n";
  }

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alt: ";
    rosidl_generator_traits::value_to_yaml(msg.alt, out);
    out << "\n";
  }

  // member: horizontal_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "horizontal_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.horizontal_accuracy, out);
    out << "\n";
  }

  // member: vertical_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vertical_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.vertical_accuracy, out);
    out << "\n";
  }

  // member: vel_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_n: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_n, out);
    out << "\n";
  }

  // member: vel_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_e: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_e, out);
    out << "\n";
  }

  // member: vel_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_d: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_d, out);
    out << "\n";
  }

  // member: speed_accuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_accuracy: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_accuracy, out);
    out << "\n";
  }

  // member: gnss_unix_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_unix_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_unix_seconds, out);
    out << "\n";
  }

  // member: gnss_unix_nanos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gnss_unix_nanos: ";
    rosidl_generator_traits::value_to_yaml(msg.gnss_unix_nanos, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GNSS & msg, bool use_flow_style = false)
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
  const rosflight_msgs::msg::GNSS & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::msg::GNSS & msg)
{
  return rosflight_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::msg::GNSS>()
{
  return "rosflight_msgs::msg::GNSS";
}

template<>
inline const char * name<rosflight_msgs::msg::GNSS>()
{
  return "rosflight_msgs/msg/GNSS";
}

template<>
struct has_fixed_size<rosflight_msgs::msg::GNSS>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rosflight_msgs::msg::GNSS>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rosflight_msgs::msg::GNSS>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__TRAITS_HPP_
