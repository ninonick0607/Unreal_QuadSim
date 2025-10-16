// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/aux_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_AuxCommand_values
{
public:
  explicit Init_AuxCommand_values(::rosflight_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::AuxCommand values(::rosflight_msgs::msg::AuxCommand::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_type_array
{
public:
  explicit Init_AuxCommand_type_array(::rosflight_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  Init_AuxCommand_values type_array(::rosflight_msgs::msg::AuxCommand::_type_array_type arg)
  {
    msg_.type_array = std::move(arg);
    return Init_AuxCommand_values(msg_);
  }

private:
  ::rosflight_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_header
{
public:
  Init_AuxCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AuxCommand_type_array header(::rosflight_msgs::msg::AuxCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AuxCommand_type_array(msg_);
  }

private:
  ::rosflight_msgs::msg::AuxCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::AuxCommand>()
{
  return rosflight_msgs::msg::builder::Init_AuxCommand_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_
