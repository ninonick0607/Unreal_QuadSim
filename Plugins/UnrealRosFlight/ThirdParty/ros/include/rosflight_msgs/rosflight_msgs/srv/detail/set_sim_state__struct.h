// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosflight_msgs:srv/SetSimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_H_
#define ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "rosflight_msgs/msg/detail/sim_state__struct.h"

/// Struct defined in srv/SetSimState in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__SetSimState_Request
{
  rosflight_msgs__msg__SimState state;
} rosflight_msgs__srv__SetSimState_Request;

// Struct for a sequence of rosflight_msgs__srv__SetSimState_Request.
typedef struct rosflight_msgs__srv__SetSimState_Request__Sequence
{
  rosflight_msgs__srv__SetSimState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__SetSimState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetSimState in the package rosflight_msgs.
typedef struct rosflight_msgs__srv__SetSimState_Response
{
  bool success;
  rosidl_runtime_c__String message;
} rosflight_msgs__srv__SetSimState_Response;

// Struct for a sequence of rosflight_msgs__srv__SetSimState_Response.
typedef struct rosflight_msgs__srv__SetSimState_Response__Sequence
{
  rosflight_msgs__srv__SetSimState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosflight_msgs__srv__SetSimState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_H_
