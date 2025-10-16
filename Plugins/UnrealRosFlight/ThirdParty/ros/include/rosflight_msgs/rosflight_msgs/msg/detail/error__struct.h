// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/Error.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__STRUCT_H_

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
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Error in the package rosflight_msgs.
/**
  * For handling rosflight hard errors
 */
typedef struct rosflight_msgs__msg__Error
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String error_message;
  uint32_t error_code;
  uint32_t reset_count;
  bool rearm;
  uint32_t pc;
} rosflight_msgs__msg__Error;

// Struct for a sequence of rosflight_msgs__msg__Error.
typedef struct rosflight_msgs__msg__Error__Sequence
{
  rosflight_msgs__msg__Error * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__Error__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__ERROR__STRUCT_H_
