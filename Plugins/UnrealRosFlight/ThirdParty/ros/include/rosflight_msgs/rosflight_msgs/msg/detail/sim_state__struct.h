// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'acceleration'
#include "geometry_msgs/msg/detail/accel__struct.h"

/// Struct defined in msg/SimState in the package rosflight_msgs.
/**
  * Truth state
 */
typedef struct rosflight_msgs__msg__SimState
{
  std_msgs__msg__Header header;
  /// Position is expressed in inertial NED frame
  /// Orientation is expressed as a rotation from body to inertial
  geometry_msgs__msg__Pose pose;
  /// Inertial velocities (linear and angular) are expressed in body frame
  geometry_msgs__msg__Twist twist;
  /// Acceleration is expressed in body frame
  geometry_msgs__msg__Accel acceleration;
} rosflight_msgs__msg__SimState;

// Struct for a sequence of rosflight_msgs__msg__SimState.
typedef struct rosflight_msgs__msg__SimState__Sequence
{
  rosflight_msgs__msg__SimState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__SimState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__STRUCT_H_
