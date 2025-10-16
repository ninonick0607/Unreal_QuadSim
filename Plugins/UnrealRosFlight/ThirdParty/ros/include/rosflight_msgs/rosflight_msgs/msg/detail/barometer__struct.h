// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Barometer.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__STRUCT_H_

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

/// Struct defined in msg/Barometer in the package rosflight_msgs.
/**
  * Barometer
 */
typedef struct rosflight_msgs__msg__Barometer
{
  std_msgs__msg__Header header;
  /// m
  float altitude;
  /// Pa
  float pressure;
  /// K
  float temperature;
} rosflight_msgs__msg__Barometer;

// Struct for a sequence of rosflight_msgs__msg__Barometer.
typedef struct rosflight_msgs__msg__Barometer__Sequence
{
  rosflight_msgs__msg__Barometer * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Barometer__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__BAROMETER__STRUCT_H_
