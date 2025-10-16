// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rosflight_msgs:msg/RCRaw.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__RC_RAW__FUNCTIONS_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__RC_RAW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rosflight_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rosflight_msgs/msg/detail/rc_raw__struct.h"

/// Initialize msg/RCRaw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosflight_msgs__msg__RCRaw
 * )) before or use
 * rosflight_msgs__msg__RCRaw__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__init(rosflight_msgs__msg__RCRaw * msg);

/// Finalize msg/RCRaw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__RCRaw__fini(rosflight_msgs__msg__RCRaw * msg);

/// Create msg/RCRaw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosflight_msgs__msg__RCRaw__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
rosflight_msgs__msg__RCRaw *
rosflight_msgs__msg__RCRaw__create();

/// Destroy msg/RCRaw message.
/**
 * It calls
 * rosflight_msgs__msg__RCRaw__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__RCRaw__destroy(rosflight_msgs__msg__RCRaw * msg);

/// Check for msg/RCRaw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__are_equal(const rosflight_msgs__msg__RCRaw * lhs, const rosflight_msgs__msg__RCRaw * rhs);

/// Copy a msg/RCRaw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__copy(
  const rosflight_msgs__msg__RCRaw * input,
  rosflight_msgs__msg__RCRaw * output);

/// Initialize array of msg/RCRaw messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosflight_msgs__msg__RCRaw__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__Sequence__init(rosflight_msgs__msg__RCRaw__Sequence * array, size_t size);

/// Finalize array of msg/RCRaw messages.
/**
 * It calls
 * rosflight_msgs__msg__RCRaw__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__RCRaw__Sequence__fini(rosflight_msgs__msg__RCRaw__Sequence * array);

/// Create array of msg/RCRaw messages.
/**
 * It allocates the memory for the array and calls
 * rosflight_msgs__msg__RCRaw__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
rosflight_msgs__msg__RCRaw__Sequence *
rosflight_msgs__msg__RCRaw__Sequence__create(size_t size);

/// Destroy array of msg/RCRaw messages.
/**
 * It calls
 * rosflight_msgs__msg__RCRaw__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__RCRaw__Sequence__destroy(rosflight_msgs__msg__RCRaw__Sequence * array);

/// Check for msg/RCRaw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__Sequence__are_equal(const rosflight_msgs__msg__RCRaw__Sequence * lhs, const rosflight_msgs__msg__RCRaw__Sequence * rhs);

/// Copy an array of msg/RCRaw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__RCRaw__Sequence__copy(
  const rosflight_msgs__msg__RCRaw__Sequence * input,
  rosflight_msgs__msg__RCRaw__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__RC_RAW__FUNCTIONS_H_
