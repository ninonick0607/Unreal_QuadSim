// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:msg/OutputRaw.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/msg/detail/output_raw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rosflight_msgs__msg__OutputRaw__init(rosflight_msgs__msg__OutputRaw * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rosflight_msgs__msg__OutputRaw__fini(msg);
    return false;
  }
  // values
  return true;
}

void
rosflight_msgs__msg__OutputRaw__fini(rosflight_msgs__msg__OutputRaw * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // values
}

bool
rosflight_msgs__msg__OutputRaw__are_equal(const rosflight_msgs__msg__OutputRaw * lhs, const rosflight_msgs__msg__OutputRaw * rhs)
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
  // values
  for (size_t i = 0; i < 14; ++i) {
    if (lhs->values[i] != rhs->values[i]) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__OutputRaw__copy(
  const rosflight_msgs__msg__OutputRaw * input,
  rosflight_msgs__msg__OutputRaw * output)
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
  // values
  for (size_t i = 0; i < 14; ++i) {
    output->values[i] = input->values[i];
  }
  return true;
}

rosflight_msgs__msg__OutputRaw *
rosflight_msgs__msg__OutputRaw__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__OutputRaw * msg = (rosflight_msgs__msg__OutputRaw *)allocator.allocate(sizeof(rosflight_msgs__msg__OutputRaw), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__msg__OutputRaw));
  bool success = rosflight_msgs__msg__OutputRaw__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__msg__OutputRaw__destroy(rosflight_msgs__msg__OutputRaw * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__msg__OutputRaw__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__msg__OutputRaw__Sequence__init(rosflight_msgs__msg__OutputRaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__OutputRaw * data = NULL;

  if (size) {
    data = (rosflight_msgs__msg__OutputRaw *)allocator.zero_allocate(size, sizeof(rosflight_msgs__msg__OutputRaw), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__msg__OutputRaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__msg__OutputRaw__fini(&data[i - 1]);
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
rosflight_msgs__msg__OutputRaw__Sequence__fini(rosflight_msgs__msg__OutputRaw__Sequence * array)
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
      rosflight_msgs__msg__OutputRaw__fini(&array->data[i]);
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

rosflight_msgs__msg__OutputRaw__Sequence *
rosflight_msgs__msg__OutputRaw__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__OutputRaw__Sequence * array = (rosflight_msgs__msg__OutputRaw__Sequence *)allocator.allocate(sizeof(rosflight_msgs__msg__OutputRaw__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__msg__OutputRaw__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__msg__OutputRaw__Sequence__destroy(rosflight_msgs__msg__OutputRaw__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__msg__OutputRaw__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__msg__OutputRaw__Sequence__are_equal(const rosflight_msgs__msg__OutputRaw__Sequence * lhs, const rosflight_msgs__msg__OutputRaw__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__msg__OutputRaw__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__OutputRaw__Sequence__copy(
  const rosflight_msgs__msg__OutputRaw__Sequence * input,
  rosflight_msgs__msg__OutputRaw__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__msg__OutputRaw);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__msg__OutputRaw * data =
      (rosflight_msgs__msg__OutputRaw *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__msg__OutputRaw__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__msg__OutputRaw__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__msg__OutputRaw__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
