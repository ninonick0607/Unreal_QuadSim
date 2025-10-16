// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_

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
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Attitude in the package rosflight_msgs.
/**
  * Defines an Euler-Angle Based Attitude Message
 */
typedef struct rosflight_msgs__msg__Attitude
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Quaternion attitude;
  geometry_msgs__msg__Vector3 angular_velocity;
} rosflight_msgs__msg__Attitude;

// Struct for a sequence of rosflight_msgs__msg__Attitude.
typedef struct rosflight_msgs__msg__Attitude__Sequence
{
  rosflight_msgs__msg__Attitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Attitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
