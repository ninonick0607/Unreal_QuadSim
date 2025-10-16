// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosflight_msgs:msg/OutputRaw.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosflight_msgs/msg/detail/output_raw__rosidl_typesupport_introspection_c.h"
#include "rosflight_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosflight_msgs/msg/detail/output_raw__functions.h"
#include "rosflight_msgs/msg/detail/output_raw__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosflight_msgs__msg__OutputRaw__init(message_memory);
}

void rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_fini_function(void * message_memory)
{
  rosflight_msgs__msg__OutputRaw__fini(message_memory);
}

size_t rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__size_function__OutputRaw__values(
  const void * untyped_member)
{
  (void)untyped_member;
  return 14;
}

const void * rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_const_function__OutputRaw__values(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_function__OutputRaw__values(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__fetch_function__OutputRaw__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_const_function__OutputRaw__values(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__assign_function__OutputRaw__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_function__OutputRaw__values(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosflight_msgs__msg__OutputRaw, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    14,  // array size
    false,  // is upper bound
    offsetof(rosflight_msgs__msg__OutputRaw, values),  // bytes offset in struct
    NULL,  // default value
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__size_function__OutputRaw__values,  // size() function pointer
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_const_function__OutputRaw__values,  // get_const(index) function pointer
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__get_function__OutputRaw__values,  // get(index) function pointer
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__fetch_function__OutputRaw__values,  // fetch(index, &value) function pointer
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__assign_function__OutputRaw__values,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_members = {
  "rosflight_msgs__msg",  // message namespace
  "OutputRaw",  // message name
  2,  // number of fields
  sizeof(rosflight_msgs__msg__OutputRaw),
  rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_member_array,  // message members
  rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_init_function,  // function to initialize message memory (memory has to be allocated)
  rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_type_support_handle = {
  0,
  &rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosflight_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosflight_msgs, msg, OutputRaw)() {
  rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_type_support_handle.typesupport_identifier) {
    rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosflight_msgs__msg__OutputRaw__rosidl_typesupport_introspection_c__OutputRaw_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
