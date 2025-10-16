// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:srv/ParamGet.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__BUILDER_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/srv/detail/param_get__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamGet_Request_name
{
public:
  Init_ParamGet_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosflight_msgs::srv::ParamGet_Request name(::rosflight_msgs::srv::ParamGet_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::ParamGet_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::ParamGet_Request>()
{
  return rosflight_msgs::srv::builder::Init_ParamGet_Request_name();
}

}  // namespace rosflight_msgs


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamGet_Response_value
{
public:
  explicit Init_ParamGet_Response_value(::rosflight_msgs::srv::ParamGet_Response & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::srv::ParamGet_Response value(::rosflight_msgs::srv::ParamGet_Response::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::ParamGet_Response msg_;
};

class Init_ParamGet_Response_exists
{
public:
  Init_ParamGet_Response_exists()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamGet_Response_value exists(::rosflight_msgs::srv::ParamGet_Response::_exists_type arg)
  {
    msg_.exists = std::move(arg);
    return Init_ParamGet_Response_value(msg_);
  }

private:
  ::rosflight_msgs::srv::ParamGet_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::ParamGet_Response>()
{
  return rosflight_msgs::srv::builder::Init_ParamGet_Response_exists();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__BUILDER_HPP_
