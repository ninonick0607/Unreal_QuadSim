// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:srv/SetSimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__BUILDER_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/srv/detail/set_sim_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_SetSimState_Request_state
{
public:
  Init_SetSimState_Request_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosflight_msgs::srv::SetSimState_Request state(::rosflight_msgs::srv::SetSimState_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::SetSimState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::SetSimState_Request>()
{
  return rosflight_msgs::srv::builder::Init_SetSimState_Request_state();
}

}  // namespace rosflight_msgs


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_SetSimState_Response_message
{
public:
  explicit Init_SetSimState_Response_message(::rosflight_msgs::srv::SetSimState_Response & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::srv::SetSimState_Response message(::rosflight_msgs::srv::SetSimState_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::SetSimState_Response msg_;
};

class Init_SetSimState_Response_success
{
public:
  Init_SetSimState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetSimState_Response_message success(::rosflight_msgs::srv::SetSimState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetSimState_Response_message(msg_);
  }

private:
  ::rosflight_msgs::srv::SetSimState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::SetSimState_Response>()
{
  return rosflight_msgs::srv::builder::Init_SetSimState_Response_success();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__BUILDER_HPP_
