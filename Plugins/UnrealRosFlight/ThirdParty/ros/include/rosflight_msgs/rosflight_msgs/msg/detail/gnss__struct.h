// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:msg/GNSS.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GNSS_FIX_TYPE_NO_FIX'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_NO_FIX = 0
};

/// Constant 'GNSS_FIX_TYPE_DEAD_RECKONING_ONLY'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1
};

/// Constant 'GNSS_FIX_TYPE_2D_FIX'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_2D_FIX = 2
};

/// Constant 'GNSS_FIX_TYPE_3D_FIX'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_3D_FIX = 3
};

/// Constant 'GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4
};

/// Constant 'GNSS_FIX_TYPE_TIME_FIX_ONLY'.
enum
{
  rosflight_msgs__msg__GNSS__GNSS_FIX_TYPE_TIME_FIX_ONLY = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GNSS in the package rosflight_msgs.
/**
  * GNSS
 */
typedef struct rosflight_msgs__msg__GNSS
{
  /// Estimated ROS time at moment of measurement, timestamp of the last received packet
  std_msgs__msg__Header header;
  /// fix type, as defined in the UBX protocol, enums defined above
  uint8_t fix_type;
  uint8_t num_sat;
  /// deg DDS
  double lat;
  /// deg DDS
  double lon;
  /// m above mean sea level
  float alt;
  /// m
  float horizontal_accuracy;
  /// m
  float vertical_accuracy;
  /// m/s
  float vel_n;
  /// m/s
  float vel_e;
  /// m/s
  float vel_d;
  /// m/s
  float speed_accuracy;
  /// Reported GNSS time (aka UNIX time), seconds
  int64_t gnss_unix_seconds;
  /// Reported GNSS fractional time (aka UNIX time), nanoseconds
  int32_t gnss_unix_nanos;
} rosflight_msgs__msg__GNSS;

// Struct for a sequence of rosflight_msgs__msg__GNSS.
typedef struct rosflight_msgs__msg__GNSS__Sequence
{
  rosflight_msgs__msg__GNSS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__msg__GNSS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__GNSS__STRUCT_H_
