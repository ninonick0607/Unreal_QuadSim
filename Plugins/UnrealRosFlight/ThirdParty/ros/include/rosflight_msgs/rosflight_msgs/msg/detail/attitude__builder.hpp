// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Attitude_angular_velocity
{
public:
  explicit Init_Attitude_angular_velocity(::rosflight_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Attitude angular_velocity(::rosflight_msgs::msg::Attitude::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Attitude msg_;
};

class Init_Attitude_attitude
{
public:
  explicit Init_Attitude_attitude(::rosflight_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_angular_velocity attitude(::rosflight_msgs::msg::Attitude::_attitude_type arg)
  {
    msg_.attitude = std::move(arg);
    return Init_Attitude_angular_velocity(msg_);
  }

private:
  ::rosflight_msgs::msg::Attitude msg_;
};

class Init_Attitude_header
{
public:
  Init_Attitude_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Attitude_attitude header(::rosflight_msgs::msg::Attitude::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Attitude_attitude(msg_);
  }

private:
  ::rosflight_msgs::msg::Attitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Attitude>()
{
  return rosflight_msgs::msg::builder::Init_Attitude_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
