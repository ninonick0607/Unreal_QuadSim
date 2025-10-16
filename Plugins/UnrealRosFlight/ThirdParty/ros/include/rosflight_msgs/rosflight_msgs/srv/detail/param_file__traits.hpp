// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:srv/ParamFile.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__TRAITS_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/srv/detail/param_file__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamFile_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamFile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamFile_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosflight_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosflight_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosflight_msgs::srv::ParamFile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::ParamFile_Request & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::ParamFile_Request>()
{
  return "rosflight_msgs::srv::ParamFile_Request";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamFile_Request>()
{
  return "rosflight_msgs/srv/ParamFile_Request";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamFile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamFile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosflight_msgs::srv::ParamFile_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamFile_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamFile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamFile_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosflight_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosflight_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosflight_msgs::srv::ParamFile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::ParamFile_Response & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::ParamFile_Response>()
{
  return "rosflight_msgs::srv::ParamFile_Response";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamFile_Response>()
{
  return "rosflight_msgs/srv/ParamFile_Response";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamFile_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamFile_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosflight_msgs::srv::ParamFile_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosflight_msgs::srv::ParamFile>()
{
  return "rosflight_msgs::srv::ParamFile";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamFile>()
{
  return "rosflight_msgs/srv/ParamFile";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamFile>
  : std::integral_constant<
    bool,
    has_fixed_size<rosflight_msgs::srv::ParamFile_Request>::value &&
    has_fixed_size<rosflight_msgs::srv::ParamFile_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamFile>
  : std::integral_constant<
    bool,
    has_bounded_size<rosflight_msgs::srv::ParamFile_Request>::value &&
    has_bounded_size<rosflight_msgs::srv::ParamFile_Response>::value
  >
{
};

template<>
struct is_service<rosflight_msgs::srv::ParamFile>
  : std::true_type
{
};

template<>
struct is_service_request<rosflight_msgs::srv::ParamFile_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosflight_msgs::srv::ParamFile_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__TRAITS_HPP_
