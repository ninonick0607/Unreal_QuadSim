// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/BatteryStatus in the package rosflight_msgs.
/**
  * Simple battery telemetry
  * Values may be 0 if not supported
 */
typedef struct rosflight_msgs__msg__BatteryStatus
{
  std_msgs__msg__Header header;
  /// volts
  float voltage;
  /// amps
  float current;
} rosflight_msgs__msg__BatteryStatus;

// Struct for a sequence of rosflight_msgs__msg__BatteryStatus.
typedef struct rosflight_msgs__msg__BatteryStatus__Sequence
{
  rosflight_msgs__msg__BatteryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__BatteryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
