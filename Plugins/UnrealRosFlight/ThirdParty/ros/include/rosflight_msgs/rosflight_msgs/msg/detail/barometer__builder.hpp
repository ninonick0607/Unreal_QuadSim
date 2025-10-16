// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Barometer.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/barometer__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Barometer_temperature
{
public:
  explicit Init_Barometer_temperature(::rosflight_msgs::msg::Barometer & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Barometer temperature(::rosflight_msgs::msg::Barometer::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Barometer msg_;
};

class Init_Barometer_pressure
{
public:
  explicit Init_Barometer_pressure(::rosflight_msgs::msg::Barometer & msg)
  : msg_(msg)
  {}
  Init_Barometer_temperature pressure(::rosflight_msgs::msg::Barometer::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_Barometer_temperature(msg_);
  }

private:
  ::rosflight_msgs::msg::Barometer msg_;
};

class Init_Barometer_altitude
{
public:
  explicit Init_Barometer_altitude(::rosflight_msgs::msg::Barometer & msg)
  : msg_(msg)
  {}
  Init_Barometer_pressure altitude(::rosflight_msgs::msg::Barometer::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_Barometer_pressure(msg_);
  }

private:
  ::rosflight_msgs::msg::Barometer msg_;
};

class Init_Barometer_header
{
public:
  Init_Barometer_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Barometer_altitude header(::rosflight_msgs::msg::Barometer::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Barometer_altitude(msg_);
  }

private:
  ::rosflight_msgs::msg::Barometer msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Barometer>()
{
  return rosflight_msgs::msg::builder::Init_Barometer_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__BUILDER_HPP_
