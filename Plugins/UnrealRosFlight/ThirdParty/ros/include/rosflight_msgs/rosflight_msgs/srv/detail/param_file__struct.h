// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:srv/ParamFile.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__STRUCT_H_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ParamFile in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamFile_Request
{
  /// filename of parameter file to load from/to
  rosidl_runtime_c__String filename;
} rosflight_msgs__srv__ParamFile_Request;

// Struct for a sequence of rosflight_msgs__srv__ParamFile_Request.
typedef struct rosflight_msgs__srv__ParamFile_Request__Sequence
{
  rosflight_msgs__srv__ParamFile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamFile_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ParamFile in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamFile_Response
{
  /// whether or not the operation was successful
  bool success;
} rosflight_msgs__srv__ParamFile_Response;

// Struct for a sequence of rosflight_msgs__srv__ParamFile_Response.
typedef struct rosflight_msgs__srv__ParamFile_Response__Sequence
{
  rosflight_msgs__srv__ParamFile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamFile_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_FILE__STRUCT_H_
