// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStatus_current
{
public:
  explicit Init_BatteryStatus_current(::rosflight_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::BatteryStatus current(::rosflight_msgs::msg::BatteryStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_voltage
{
public:
  explicit Init_BatteryStatus_voltage(::rosflight_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_current voltage(::rosflight_msgs::msg::BatteryStatus::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryStatus_current(msg_);
  }

private:
  ::rosflight_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_header
{
public:
  Init_BatteryStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatus_voltage header(::rosflight_msgs::msg::BatteryStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BatteryStatus_voltage(msg_);
  }

private:
  ::rosflight_msgs::msg::BatteryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::BatteryStatus>()
{
  return rosflight_msgs::msg::builder::Init_BatteryStatus_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
