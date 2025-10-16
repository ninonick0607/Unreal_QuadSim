// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_

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

/// Struct defined in msg/Airspeed in the package rosflight_msgs.
/**
  * Airspeed
 */
typedef struct rosflight_msgs__msg__Airspeed
{
  std_msgs__msg__Header header;
  /// m/s
  float velocity;
  /// Pa
  float differential_pressure;
  /// K
  float temperature;
} rosflight_msgs__msg__Airspeed;

// Struct for a sequence of rosflight_msgs__msg__Airspeed.
typedef struct rosflight_msgs__msg__Airspeed__Sequence
{
  rosflight_msgs__msg__Airspeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Airspeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_
