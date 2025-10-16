// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:srv/ParamSet.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__TRAITS_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/srv/detail/param_set__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamSet_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamSet_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamSet_Request & msg, bool use_flow_style = false)
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
  const rosflight_msgs::srv::ParamSet_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::ParamSet_Request & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::ParamSet_Request>()
{
  return "rosflight_msgs::srv::ParamSet_Request";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamSet_Request>()
{
  return "rosflight_msgs/srv/ParamSet_Request";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamSet_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamSet_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosflight_msgs::srv::ParamSet_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ParamSet_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: exists
  {
    out << "exists: ";
    rosidl_generator_traits::value_to_yaml(msg.exists, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamSet_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: exists
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "exists: ";
    rosidl_generator_traits::value_to_yaml(msg.exists, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamSet_Response & msg, bool use_flow_style = false)
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
  const rosflight_msgs::srv::ParamSet_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::ParamSet_Response & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::ParamSet_Response>()
{
  return "rosflight_msgs::srv::ParamSet_Response";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamSet_Response>()
{
  return "rosflight_msgs/srv/ParamSet_Response";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamSet_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamSet_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosflight_msgs::srv::ParamSet_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosflight_msgs::srv::ParamSet>()
{
  return "rosflight_msgs::srv::ParamSet";
}

template<>
inline const char * name<rosflight_msgs::srv::ParamSet>()
{
  return "rosflight_msgs/srv/ParamSet";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::ParamSet>
  : std::integral_constant<
    bool,
    has_fixed_size<rosflight_msgs::srv::ParamSet_Request>::value &&
    has_fixed_size<rosflight_msgs::srv::ParamSet_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosflight_msgs::srv::ParamSet>
  : std::integral_constant<
    bool,
    has_bounded_size<rosflight_msgs::srv::ParamSet_Request>::value &&
    has_bounded_size<rosflight_msgs::srv::ParamSet_Response>::value
  >
{
};

template<>
struct is_service<rosflight_msgs::srv::ParamSet>
  : std::true_type
{
};

template<>
struct is_service_request<rosflight_msgs::srv::ParamSet_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosflight_msgs::srv::ParamSet_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__TRAITS_HPP_
