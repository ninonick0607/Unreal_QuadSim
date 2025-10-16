// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:srv/ParamFile.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__BUILDER_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/srv/detail/param_file__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamFile_Request_filename
{
public:
  Init_ParamFile_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosflight_msgs::srv::ParamFile_Request filename(::rosflight_msgs::srv::ParamFile_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::ParamFile_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::ParamFile_Request>()
{
  return rosflight_msgs::srv::builder::Init_ParamFile_Request_filename();
}

}  // namespace rosflight_msgs


namespace rosflight_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamFile_Response_success
{
public:
  Init_ParamFile_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosflight_msgs::srv::ParamFile_Response success(::rosflight_msgs::srv::ParamFile_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::srv::ParamFile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::srv::ParamFile_Response>()
{
  return rosflight_msgs::srv::builder::Init_ParamFile_Response_success();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__BUILDER_HPP_
