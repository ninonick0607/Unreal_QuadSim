// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_loop_time_us
{
public:
  explicit Init_Status_loop_time_us(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Status loop_time_us(::rosflight_msgs::msg::Status::_loop_time_us_type arg)
  {
    msg_.loop_time_us = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_num_errors
{
public:
  explicit Init_Status_num_errors(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_loop_time_us num_errors(::rosflight_msgs::msg::Status::_num_errors_type arg)
  {
    msg_.num_errors = std::move(arg);
    return Init_Status_loop_time_us(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_error_code
{
public:
  explicit Init_Status_error_code(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_num_errors error_code(::rosflight_msgs::msg::Status::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_Status_num_errors(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_control_mode
{
public:
  explicit Init_Status_control_mode(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_error_code control_mode(::rosflight_msgs::msg::Status::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_Status_error_code(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_offboard
{
public:
  explicit Init_Status_offboard(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_control_mode offboard(::rosflight_msgs::msg::Status::_offboard_type arg)
  {
    msg_.offboard = std::move(arg);
    return Init_Status_control_mode(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_rc_override
{
public:
  explicit Init_Status_rc_override(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_offboard rc_override(::rosflight_msgs::msg::Status::_rc_override_type arg)
  {
    msg_.rc_override = std::move(arg);
    return Init_Status_offboard(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_failsafe
{
public:
  explicit Init_Status_failsafe(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_rc_override failsafe(::rosflight_msgs::msg::Status::_failsafe_type arg)
  {
    msg_.failsafe = std::move(arg);
    return Init_Status_rc_override(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_armed
{
public:
  explicit Init_Status_armed(::rosflight_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_failsafe armed(::rosflight_msgs::msg::Status::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_Status_failsafe(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_armed header(::rosflight_msgs::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_armed(msg_);
  }

private:
  ::rosflight_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Status>()
{
  return rosflight_msgs::msg::builder::Init_Status_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
