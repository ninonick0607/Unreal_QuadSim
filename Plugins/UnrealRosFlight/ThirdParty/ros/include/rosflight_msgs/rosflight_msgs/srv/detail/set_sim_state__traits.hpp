// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosflight_msgs:srv/SetSimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__TRAITS_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosflight_msgs/srv/detail/set_sim_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'state'
#include "rosflight_msgs/msg/detail/sim_state__traits.hpp"

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSimState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    to_flow_style_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSimState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state:\n";
    to_block_style_yaml(msg.state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSimState_Request & msg, bool use_flow_style = false)
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
  const rosflight_msgs::srv::SetSimState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::SetSimState_Request & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::SetSimState_Request>()
{
  return "rosflight_msgs::srv::SetSimState_Request";
}

template<>
inline const char * name<rosflight_msgs::srv::SetSimState_Request>()
{
  return "rosflight_msgs/srv/SetSimState_Request";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::SetSimState_Request>
  : std::integral_constant<bool, has_fixed_size<rosflight_msgs::msg::SimState>::value> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::SetSimState_Request>
  : std::integral_constant<bool, has_bounded_size<rosflight_msgs::msg::SimState>::value> {};

template<>
struct is_message<rosflight_msgs::srv::SetSimState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosflight_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSimState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSimState_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSimState_Response & msg, bool use_flow_style = false)
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
  const rosflight_msgs::srv::SetSimState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosflight_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosflight_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosflight_msgs::srv::SetSimState_Response & msg)
{
  return rosflight_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosflight_msgs::srv::SetSimState_Response>()
{
  return "rosflight_msgs::srv::SetSimState_Response";
}

template<>
inline const char * name<rosflight_msgs::srv::SetSimState_Response>()
{
  return "rosflight_msgs/srv/SetSimState_Response";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::SetSimState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosflight_msgs::srv::SetSimState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosflight_msgs::srv::SetSimState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosflight_msgs::srv::SetSimState>()
{
  return "rosflight_msgs::srv::SetSimState";
}

template<>
inline const char * name<rosflight_msgs::srv::SetSimState>()
{
  return "rosflight_msgs/srv/SetSimState";
}

template<>
struct has_fixed_size<rosflight_msgs::srv::SetSimState>
  : std::integral_constant<
    bool,
    has_fixed_size<rosflight_msgs::srv::SetSimState_Request>::value &&
    has_fixed_size<rosflight_msgs::srv::SetSimState_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosflight_msgs::srv::SetSimState>
  : std::integral_constant<
    bool,
    has_bounded_size<rosflight_msgs::srv::SetSimState_Request>::value &&
    has_bounded_size<rosflight_msgs::srv::SetSimState_Response>::value
  >
{
};

template<>
struct is_service<rosflight_msgs::srv::SetSimState>
  : std::true_type
{
};

template<>
struct is_service_request<rosflight_msgs::srv::SetSimState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosflight_msgs::srv::SetSimState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__TRAITS_HPP_
