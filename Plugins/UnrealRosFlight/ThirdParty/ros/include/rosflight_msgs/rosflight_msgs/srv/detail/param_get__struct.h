// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:srv/ParamGet.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ParamGet in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamGet_Request
{
  /// the name of the parameter value to retrieve
  rosidl_runtime_c__String name;
} rosflight_msgs__srv__ParamGet_Request;

// Struct for a sequence of rosflight_msgs__srv__ParamGet_Request.
typedef struct rosflight_msgs__srv__ParamGet_Request__Sequence
{
  rosflight_msgs__srv__ParamGet_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamGet_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ParamGet in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamGet_Response
{
  /// whether the request parameter exists
  bool exists;
  /// the value of the requested parameter
  double value;
} rosflight_msgs__srv__ParamGet_Response;

// Struct for a sequence of rosflight_msgs__srv__ParamGet_Response.
typedef struct rosflight_msgs__srv__ParamGet_Response__Sequence
{
  rosflight_msgs__srv__ParamGet_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamGet_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_GET__STRUCT_H_
