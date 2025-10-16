// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/OutputRaw.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__OUTPUT_RAW__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__OUTPUT_RAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/output_raw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_OutputRaw_values
{
public:
  explicit Init_OutputRaw_values(::rosflight_msgs::msg::OutputRaw & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::OutputRaw values(::rosflight_msgs::msg::OutputRaw::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::OutputRaw msg_;
};

class Init_OutputRaw_header
{
public:
  Init_OutputRaw_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OutputRaw_values header(::rosflight_msgs::msg::OutputRaw::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OutputRaw_values(msg_);
  }

private:
  ::rosflight_msgs::msg::OutputRaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::OutputRaw>()
{
  return rosflight_msgs::msg::builder::Init_OutputRaw_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__OUTPUT_RAW__BUILDER_HPP_
