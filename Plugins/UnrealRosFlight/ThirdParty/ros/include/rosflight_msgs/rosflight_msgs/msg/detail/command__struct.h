// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Command.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_PASS_THROUGH'.
/**
  * control mode flags
 */
enum
{
  rosflight_msgs__msg__Command__MODE_PASS_THROUGH = 0
};

/// Constant 'MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE'.
enum
{
  rosflight_msgs__msg__Command__MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 1
};

/// Constant 'MODE_ROLL_PITCH_YAWRATE_THROTTLE'.
enum
{
  rosflight_msgs__msg__Command__MODE_ROLL_PITCH_YAWRATE_THROTTLE = 2
};

/// Constant 'IGNORE_NONE'.
/**
  * ignore field bitmasks
 */
enum
{
  rosflight_msgs__msg__Command__IGNORE_NONE = 0
};

/// Constant 'IGNORE_QX'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_QX = 1
};

/// Constant 'IGNORE_QY'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_QY = 2
};

/// Constant 'IGNORE_QZ'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_QZ = 4
};

/// Constant 'IGNORE_FX'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_FX = 8
};

/// Constant 'IGNORE_FY'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_FY = 16
};

/// Constant 'IGNORE_FZ'.
enum
{
  rosflight_msgs__msg__Command__IGNORE_FZ = 32
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Command in the package rosflight_msgs.
/**
  * Offboard control command message
 */
typedef struct rosflight_msgs__msg__Command
{
  std_msgs__msg__Header header;
  /// offboard control mode for interpreting value fields
  uint8_t mode;
  /// bitmask for ignore specific setpoint values
  uint8_t ignore;
  float qx;
  float qy;
  float qz;
  float fx;
  float fy;
  float fz;
} rosflight_msgs__msg__Command;

// Struct for a sequence of rosflight_msgs__msg__Command.
typedef struct rosflight_msgs__msg__Command__Sequence
{
  rosflight_msgs__msg__Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__COMMAND__STRUCT_H_
