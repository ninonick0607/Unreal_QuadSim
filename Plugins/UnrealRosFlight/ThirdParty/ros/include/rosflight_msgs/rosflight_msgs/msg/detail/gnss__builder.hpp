// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosflight_msgs:msg/GNSS.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__BUILDER_HPP_
#define ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosflight_msgs/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosflight_msgs
{

namespace msg
{

namespace builder
{

class Init_GNSS_gnss_unix_nanos
{
public:
  explicit Init_GNSS_gnss_unix_nanos(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  ::rosflight_msgs::msg::GNSS gnss_unix_nanos(::rosflight_msgs::msg::GNSS::_gnss_unix_nanos_type arg)
  {
    msg_.gnss_unix_nanos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_gnss_unix_seconds
{
public:
  explicit Init_GNSS_gnss_unix_seconds(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_gnss_unix_nanos gnss_unix_seconds(::rosflight_msgs::msg::GNSS::_gnss_unix_seconds_type arg)
  {
    msg_.gnss_unix_seconds = std::move(arg);
    return Init_GNSS_gnss_unix_nanos(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_speed_accuracy
{
public:
  explicit Init_GNSS_speed_accuracy(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_gnss_unix_seconds speed_accuracy(::rosflight_msgs::msg::GNSS::_speed_accuracy_type arg)
  {
    msg_.speed_accuracy = std::move(arg);
    return Init_GNSS_gnss_unix_seconds(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_vel_d
{
public:
  explicit Init_GNSS_vel_d(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_speed_accuracy vel_d(::rosflight_msgs::msg::GNSS::_vel_d_type arg)
  {
    msg_.vel_d = std::move(arg);
    return Init_GNSS_speed_accuracy(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_vel_e
{
public:
  explicit Init_GNSS_vel_e(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_vel_d vel_e(::rosflight_msgs::msg::GNSS::_vel_e_type arg)
  {
    msg_.vel_e = std::move(arg);
    return Init_GNSS_vel_d(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_vel_n
{
public:
  explicit Init_GNSS_vel_n(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_vel_e vel_n(::rosflight_msgs::msg::GNSS::_vel_n_type arg)
  {
    msg_.vel_n = std::move(arg);
    return Init_GNSS_vel_e(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_vertical_accuracy
{
public:
  explicit Init_GNSS_vertical_accuracy(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_vel_n vertical_accuracy(::rosflight_msgs::msg::GNSS::_vertical_accuracy_type arg)
  {
    msg_.vertical_accuracy = std::move(arg);
    return Init_GNSS_vel_n(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_horizontal_accuracy
{
public:
  explicit Init_GNSS_horizontal_accuracy(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_vertical_accuracy horizontal_accuracy(::rosflight_msgs::msg::GNSS::_horizontal_accuracy_type arg)
  {
    msg_.horizontal_accuracy = std::move(arg);
    return Init_GNSS_vertical_accuracy(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_alt
{
public:
  explicit Init_GNSS_alt(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_horizontal_accuracy alt(::rosflight_msgs::msg::GNSS::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return Init_GNSS_horizontal_accuracy(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_lon
{
public:
  explicit Init_GNSS_lon(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_alt lon(::rosflight_msgs::msg::GNSS::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_GNSS_alt(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_lat
{
public:
  explicit Init_GNSS_lat(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_lon lat(::rosflight_msgs::msg::GNSS::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_GNSS_lon(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_num_sat
{
public:
  explicit Init_GNSS_num_sat(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_lat num_sat(::rosflight_msgs::msg::GNSS::_num_sat_type arg)
  {
    msg_.num_sat = std::move(arg);
    return Init_GNSS_lat(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_fix_type
{
public:
  explicit Init_GNSS_fix_type(::rosflight_msgs::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_num_sat fix_type(::rosflight_msgs::msg::GNSS::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_GNSS_num_sat(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

class Init_GNSS_header
{
public:
  Init_GNSS_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSS_fix_type header(::rosflight_msgs::msg::GNSS::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GNSS_fix_type(msg_);
  }

private:
  ::rosflight_msgs::msg::GNSS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosflight_msgs::msg::GNSS>()
{
  return rosflight_msgs::msg::builder::Init_GNSS_header();
}

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__BUILDER_HPP_
