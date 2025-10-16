// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

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

/// Struct defined in msg/Status in the package rosflight_msgs.
/**
  * Flight controller status message
 */
typedef struct rosflight_msgs__msg__Status
{
  std_msgs__msg__Header header;
  /// True if armed
  bool armed;
  /// True if in failsafe
  bool failsafe;
  /// True if RC is in control
  bool rc_override;
  /// True if offboard control is active
  bool offboard;
  /// Onboard control mode
  uint8_t control_mode;
  /// Onboard error code
  uint8_t error_code;
  /// Number of errors
  int16_t num_errors;
  /// Loop time in microseconds
  int16_t loop_time_us;
} rosflight_msgs__msg__Status;

// Struct for a sequence of rosflight_msgs__msg__Status.
typedef struct rosflight_msgs__msg__Status__Sequence
{
  rosflight_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
