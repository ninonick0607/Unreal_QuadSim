// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rosflight_msgs:msg/SimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__FUNCTIONS_H_
#define ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rosflight_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rosflight_msgs/msg/detail/sim_state__struct.h"

/// Initialize msg/SimState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosflight_msgs__msg__SimState
 * )) before or use
 * rosflight_msgs__msg__SimState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__SimState__init(rosflight_msgs__msg__SimState * msg);

/// Finalize msg/SimState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__SimState__fini(rosflight_msgs__msg__SimState * msg);

/// Create msg/SimState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosflight_msgs__msg__SimState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
rosflight_msgs__msg__SimState *
rosflight_msgs__msg__SimState__create();

/// Destroy msg/SimState message.
/**
 * It calls
 * rosflight_msgs__msg__SimState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__SimState__destroy(rosflight_msgs__msg__SimState * msg);

/// Check for msg/SimState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__SimState__are_equal(const rosflight_msgs__msg__SimState * lhs, const rosflight_msgs__msg__SimState * rhs);

/// Copy a msg/SimState message.
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
rosflight_msgs__msg__SimState__copy(
  const rosflight_msgs__msg__SimState * input,
  rosflight_msgs__msg__SimState * output);

/// Initialize array of msg/SimState messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosflight_msgs__msg__SimState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__SimState__Sequence__init(rosflight_msgs__msg__SimState__Sequence * array, size_t size);

/// Finalize array of msg/SimState messages.
/**
 * It calls
 * rosflight_msgs__msg__SimState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__SimState__Sequence__fini(rosflight_msgs__msg__SimState__Sequence * array);

/// Create array of msg/SimState messages.
/**
 * It allocates the memory for the array and calls
 * rosflight_msgs__msg__SimState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
rosflight_msgs__msg__SimState__Sequence *
rosflight_msgs__msg__SimState__Sequence__create(size_t size);

/// Destroy array of msg/SimState messages.
/**
 * It calls
 * rosflight_msgs__msg__SimState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
void
rosflight_msgs__msg__SimState__Sequence__destroy(rosflight_msgs__msg__SimState__Sequence * array);

/// Check for msg/SimState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosflight_msgs
bool
rosflight_msgs__msg__SimState__Sequence__are_equal(const rosflight_msgs__msg__SimState__Sequence * lhs, const rosflight_msgs__msg__SimState__Sequence * rhs);

/// Copy an array of msg/SimState messages.
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
rosflight_msgs__msg__SimState__Sequence__copy(
  const rosflight_msgs__msg__SimState__Sequence * input,
  rosflight_msgs__msg__SimState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSFLIGHT_MSGS__MSG__DETAIL__SIM_STATE__FUNCTIONS_H_
