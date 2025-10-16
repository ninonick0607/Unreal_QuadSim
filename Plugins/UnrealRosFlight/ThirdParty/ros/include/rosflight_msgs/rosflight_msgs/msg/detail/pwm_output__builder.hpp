// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/PwmOutput.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__PWM_OUTPUT__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__PWM_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/pwm_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_PwmOutput_values
{
public:
  explicit Init_PwmOutput_values(::rosflight_msgs::msg::PwmOutput & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::PwmOutput values(::rosflight_msgs::msg::PwmOutput::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::PwmOutput msg_;
};

class Init_PwmOutput_header
{
public:
  Init_PwmOutput_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PwmOutput_values header(::rosflight_msgs::msg::PwmOutput::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PwmOutput_values(msg_);
  }

private:
  ::rosflight_msgs::msg::PwmOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::PwmOutput>()
{
  return rosflight_msgs::msg::builder::Init_PwmOutput_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__PWM_OUTPUT__BUILDER_HPP_
