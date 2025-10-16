// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/sim_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_SimState_acceleration
{
public:
  explicit Init_SimState_acceleration(::rosflight_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::SimState acceleration(::rosflight_msgs::msg::SimState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::SimState msg_;
};

class Init_SimState_twist
{
public:
  explicit Init_SimState_twist(::rosflight_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_acceleration twist(::rosflight_msgs::msg::SimState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_SimState_acceleration(msg_);
  }

private:
  ::rosflight_msgs::msg::SimState msg_;
};

class Init_SimState_pose
{
public:
  explicit Init_SimState_pose(::rosflight_msgs::msg::SimState & msg)
  : msg_(msg)
  {}
  Init_SimState_twist pose(::rosflight_msgs::msg::SimState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_SimState_twist(msg_);
  }

private:
  ::rosflight_msgs::msg::SimState msg_;
};

class Init_SimState_header
{
public:
  Init_SimState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SimState_pose header(::rosflight_msgs::msg::SimState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SimState_pose(msg_);
  }

private:
  ::rosflight_msgs::msg::SimState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::SimState>()
{
  return rosflight_msgs::msg::builder::Init_SimState_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__BUILDER_HPP_
