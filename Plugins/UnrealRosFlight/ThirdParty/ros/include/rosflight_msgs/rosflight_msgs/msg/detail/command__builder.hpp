// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Command_fz
{
public:
  explicit Init_Command_fz(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Command fz(::rosflight_msgs::msg::Command::_fz_type arg)
  {
    msg_.fz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_fy
{
public:
  explicit Init_Command_fy(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_fz fy(::rosflight_msgs::msg::Command::_fy_type arg)
  {
    msg_.fy = std::move(arg);
    return Init_Command_fz(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_fx
{
public:
  explicit Init_Command_fx(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_fy fx(::rosflight_msgs::msg::Command::_fx_type arg)
  {
    msg_.fx = std::move(arg);
    return Init_Command_fy(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_qz
{
public:
  explicit Init_Command_qz(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_fx qz(::rosflight_msgs::msg::Command::_qz_type arg)
  {
    msg_.qz = std::move(arg);
    return Init_Command_fx(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_qy
{
public:
  explicit Init_Command_qy(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_qz qy(::rosflight_msgs::msg::Command::_qy_type arg)
  {
    msg_.qy = std::move(arg);
    return Init_Command_qz(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_qx
{
public:
  explicit Init_Command_qx(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_qy qx(::rosflight_msgs::msg::Command::_qx_type arg)
  {
    msg_.qx = std::move(arg);
    return Init_Command_qy(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_ignore
{
public:
  explicit Init_Command_ignore(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_qx ignore(::rosflight_msgs::msg::Command::_ignore_type arg)
  {
    msg_.ignore = std::move(arg);
    return Init_Command_qx(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_mode
{
public:
  explicit Init_Command_mode(::rosflight_msgs::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_ignore mode(::rosflight_msgs::msg::Command::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Command_ignore(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

class Init_Command_header
{
public:
  Init_Command_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_mode header(::rosflight_msgs::msg::Command::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Command_mode(msg_);
  }

private:
  ::rosflight_msgs::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Command>()
{
  return rosflight_msgs::msg::builder::Init_Command_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__BUILDER_HPP_
