// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rosflight_msgs__msg__Status__init(rosflight_msgs__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rosflight_msgs__msg__Status__fini(msg);
    return false;
  }
  // armed
  // failsafe
  // rc_override
  // offboard
  // control_mode
  // error_code
  // num_errors
  // loop_time_us
  return true;
}

void
rosflight_msgs__msg__Status__fini(rosflight_msgs__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // armed
  // failsafe
  // rc_override
  // offboard
  // control_mode
  // error_code
  // num_errors
  // loop_time_us
}

bool
rosflight_msgs__msg__Status__are_equal(const rosflight_msgs__msg__Status * lhs, const rosflight_msgs__msg__Status * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // armed
  if (lhs->armed != rhs->armed) {
    return false;
  }
  // failsafe
  if (lhs->failsafe != rhs->failsafe) {
    return false;
  }
  // rc_override
  if (lhs->rc_override != rhs->rc_override) {
    return false;
  }
  // offboard
  if (lhs->offboard != rhs->offboard) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // num_errors
  if (lhs->num_errors != rhs->num_errors) {
    return false;
  }
  // loop_time_us
  if (lhs->loop_time_us != rhs->loop_time_us) {
    return false;
  }
  return true;
}

bool
rosflight_msgs__msg__Status__copy(
  const rosflight_msgs__msg__Status * input,
  rosflight_msgs__msg__Status * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // armed
  output->armed = input->armed;
  // failsafe
  output->failsafe = input->failsafe;
  // rc_override
  output->rc_override = input->rc_override;
  // offboard
  output->offboard = input->offboard;
  // control_mode
  output->control_mode = input->control_mode;
  // error_code
  output->error_code = input->error_code;
  // num_errors
  output->num_errors = input->num_errors;
  // loop_time_us
  output->loop_time_us = input->loop_time_us;
  return true;
}

rosflight_msgs__msg__Status *
rosflight_msgs__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Status * msg = (rosflight_msgs__msg__Status *)allocator.allocate(sizeof(rosflight_msgs__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__msg__Status));
  bool success = rosflight_msgs__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__msg__Status__destroy(rosflight_msgs__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__msg__Status__Sequence__init(rosflight_msgs__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Status * data = NULL;

  if (size) {
    data = (rosflight_msgs__msg__Status *)allocator.zero_allocate(size, sizeof(rosflight_msgs__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__msg__Status__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosflight_msgs__msg__Status__Sequence__fini(rosflight_msgs__msg__Status__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosflight_msgs__msg__Status__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosflight_msgs__msg__Status__Sequence *
rosflight_msgs__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Status__Sequence * array = (rosflight_msgs__msg__Status__Sequence *)allocator.allocate(sizeof(rosflight_msgs__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__msg__Status__Sequence__destroy(rosflight_msgs__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__msg__Status__Sequence__are_equal(const rosflight_msgs__msg__Status__Sequence * lhs, const rosflight_msgs__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__Status__Sequence__copy(
  const rosflight_msgs__msg__Status__Sequence * input,
  rosflight_msgs__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__msg__Status * data =
      (rosflight_msgs__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
