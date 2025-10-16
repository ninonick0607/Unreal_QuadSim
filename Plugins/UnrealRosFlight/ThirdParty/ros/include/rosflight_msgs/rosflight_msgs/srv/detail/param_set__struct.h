// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:srv/ParamSet.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_H_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_H_

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

/// Struct defined in srv/ParamSet in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamSet_Request
{
  /// the name of the parameter to set
  rosidl_runtime_c__String name;
  /// the value to set the parameter to
  double value;
} rosflight_msgs__srv__ParamSet_Request;

// Struct for a sequence of rosflight_msgs__srv__ParamSet_Request.
typedef struct rosflight_msgs__srv__ParamSet_Request__Sequence
{
  rosflight_msgs__srv__ParamSet_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamSet_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ParamSet in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__ParamSet_Response
{
  /// whether the requested parameter exists
  bool exists;
} rosflight_msgs__srv__ParamSet_Response;

// Struct for a sequence of rosflight_msgs__srv__ParamSet_Response.
typedef struct rosflight_msgs__srv__ParamSet_Response__Sequence
{
  rosflight_msgs__srv__ParamSet_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__ParamSet_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_H_
