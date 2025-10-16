// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'AUX_COMMAND_DISABLED'.
/**
  * Command Types
 */
enum
{
  rosflight_msgs__msg__AuxCommand__AUX_COMMAND_DISABLED = 0
};

/// Constant 'AUX_COMMAND_SERVO'.
enum
{
  rosflight_msgs__msg__AuxCommand__AUX_COMMAND_SERVO = 1
};

/// Constant 'AUX_COMMAND_MOTOR'.
enum
{
  rosflight_msgs__msg__AuxCommand__AUX_COMMAND_MOTOR = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/AuxCommand in the package rosflight_msgs.
/**
  * Auxilliary Override Command Message
 */
typedef struct rosflight_msgs__msg__AuxCommand
{
  std_msgs__msg__Header header;
  uint8_t type_array[14];
  float values[14];
} rosflight_msgs__msg__AuxCommand;

// Struct for a sequence of rosflight_msgs__msg__AuxCommand.
typedef struct rosflight_msgs__msg__AuxCommand__Sequence
{
  rosflight_msgs__msg__AuxCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__AuxCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_
