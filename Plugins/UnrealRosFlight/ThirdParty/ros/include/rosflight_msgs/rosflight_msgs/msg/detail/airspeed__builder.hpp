// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/airspeed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_Airspeed_temperature
{
public:
  explicit Init_Airspeed_temperature(::rosflight_msgs::msg::Airspeed & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::Airspeed temperature(::rosflight_msgs::msg::Airspeed::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::Airspeed msg_;
};

class Init_Airspeed_differential_pressure
{
public:
  explicit Init_Airspeed_differential_pressure(::rosflight_msgs::msg::Airspeed & msg)
  : msg_(msg)
  {}
  Init_Airspeed_temperature differential_pressure(::rosflight_msgs::msg::Airspeed::_differential_pressure_type arg)
  {
    msg_.differential_pressure = std::move(arg);
    return Init_Airspeed_temperature(msg_);
  }

private:
  ::rosflight_msgs::msg::Airspeed msg_;
};

class Init_Airspeed_velocity
{
public:
  explicit Init_Airspeed_velocity(::rosflight_msgs::msg::Airspeed & msg)
  : msg_(msg)
  {}
  Init_Airspeed_differential_pressure velocity(::rosflight_msgs::msg::Airspeed::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Airspeed_differential_pressure(msg_);
  }

private:
  ::rosflight_msgs::msg::Airspeed msg_;
};

class Init_Airspeed_header
{
public:
  Init_Airspeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Airspeed_velocity header(::rosflight_msgs::msg::Airspeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Airspeed_velocity(msg_);
  }

private:
  ::rosflight_msgs::msg::Airspeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::Airspeed>()
{
  return rosflight_msgs::msg::builder::Init_Airspeed_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__BUILDER_HPP_
