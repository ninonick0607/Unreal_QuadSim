// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Error_pc
{
public:
  explicit Init_Error_pc(::rosflight_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Error pc(::rosflight_msgs::msg::Error::_pc_type arg)
  {
    msg_.pc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

class Init_Error_rearm
{
public:
  explicit Init_Error_rearm(::rosflight_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  Init_Error_pc rearm(::rosflight_msgs::msg::Error::_rearm_type arg)
  {
    msg_.rearm = std::move(arg);
    return Init_Error_pc(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

class Init_Error_reset_count
{
public:
  explicit Init_Error_reset_count(::rosflight_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  Init_Error_rearm reset_count(::rosflight_msgs::msg::Error::_reset_count_type arg)
  {
    msg_.reset_count = std::move(arg);
    return Init_Error_rearm(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

class Init_Error_error_code
{
public:
  explicit Init_Error_error_code(::rosflight_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  Init_Error_reset_count error_code(::rosflight_msgs::msg::Error::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_Error_reset_count(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

class Init_Error_error_message
{
public:
  explicit Init_Error_error_message(::rosflight_msgs::msg::Error & msg)
  : msg_(msg)
  {}
  Init_Error_error_code error_message(::rosflight_msgs::msg::Error::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_Error_error_code(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

class Init_Error_header
{
public:
  Init_Error_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Error_error_message header(::rosflight_msgs::msg::Error::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Error_error_message(msg_);
  }

private:
  ::rosflight_msgs::msg::Error msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Error>()
{
  return rosflight_msgs::msg::builder::Init_Error_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__BUILDER_HPP_
