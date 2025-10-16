// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:msg/Error.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/msg/detail/error__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
rosflight_msgs__msg__Error__init(rosflight_msgs__msg__Error * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rosflight_msgs__msg__Error__fini(msg);
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    rosflight_msgs__msg__Error__fini(msg);
    return false;
  }
  // error_code
  // reset_count
  // rearm
  // pc
  return true;
}

void
rosflight_msgs__msg__Error__fini(rosflight_msgs__msg__Error * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // error_code
  // reset_count
  // rearm
  // pc
}

bool
rosflight_msgs__msg__Error__are_equal(const rosflight_msgs__msg__Error * lhs, const rosflight_msgs__msg__Error * rhs)
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
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // reset_count
  if (lhs->reset_count != rhs->reset_count) {
    return false;
  }
  // rearm
  if (lhs->rearm != rhs->rearm) {
    return false;
  }
  // pc
  if (lhs->pc != rhs->pc) {
    return false;
  }
  return true;
}

bool
rosflight_msgs__msg__Error__copy(
  const rosflight_msgs__msg__Error * input,
  rosflight_msgs__msg__Error * output)
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
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // error_code
  output->error_code = input->error_code;
  // reset_count
  output->reset_count = input->reset_count;
  // rearm
  output->rearm = input->rearm;
  // pc
  output->pc = input->pc;
  return true;
}

rosflight_msgs__msg__Error *
rosflight_msgs__msg__Error__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Error * msg = (rosflight_msgs__msg__Error *)allocator.allocate(sizeof(rosflight_msgs__msg__Error), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__msg__Error));
  bool success = rosflight_msgs__msg__Error__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__msg__Error__destroy(rosflight_msgs__msg__Error * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__msg__Error__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__msg__Error__Sequence__init(rosflight_msgs__msg__Error__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Error * data = NULL;

  if (size) {
    data = (rosflight_msgs__msg__Error *)allocator.zero_allocate(size, sizeof(rosflight_msgs__msg__Error), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__msg__Error__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__msg__Error__fini(&data[i - 1]);
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
rosflight_msgs__msg__Error__Sequence__fini(rosflight_msgs__msg__Error__Sequence * array)
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
      rosflight_msgs__msg__Error__fini(&array->data[i]);
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

rosflight_msgs__msg__Error__Sequence *
rosflight_msgs__msg__Error__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__Error__Sequence * array = (rosflight_msgs__msg__Error__Sequence *)allocator.allocate(sizeof(rosflight_msgs__msg__Error__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__msg__Error__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__msg__Error__Sequence__destroy(rosflight_msgs__msg__Error__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__msg__Error__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__msg__Error__Sequence__are_equal(const rosflight_msgs__msg__Error__Sequence * lhs, const rosflight_msgs__msg__Error__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__msg__Error__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__Error__Sequence__copy(
  const rosflight_msgs__msg__Error__Sequence * input,
  rosflight_msgs__msg__Error__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__msg__Error);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__msg__Error * data =
      (rosflight_msgs__msg__Error *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__msg__Error__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__msg__Error__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__msg__Error__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
