// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:msg/RCRaw.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/msg/detail/rc_raw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rosflight_msgs__msg__RCRaw__init(rosflight_msgs__msg__RCRaw * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rosflight_msgs__msg__RCRaw__fini(msg);
    return false;
  }
  // values
  return true;
}

void
rosflight_msgs__msg__RCRaw__fini(rosflight_msgs__msg__RCRaw * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // values
}

bool
rosflight_msgs__msg__RCRaw__are_equal(const rosflight_msgs__msg__RCRaw * lhs, const rosflight_msgs__msg__RCRaw * rhs)
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
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->values[i] != rhs->values[i]) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__RCRaw__copy(
  const rosflight_msgs__msg__RCRaw * input,
  rosflight_msgs__msg__RCRaw * output)
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
  for (size_t i = 0; i < 8; ++i) {
    output->values[i] = input->values[i];
  }
  return true;
}

rosflight_msgs__msg__RCRaw *
rosflight_msgs__msg__RCRaw__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__RCRaw * msg = (rosflight_msgs__msg__RCRaw *)allocator.allocate(sizeof(rosflight_msgs__msg__RCRaw), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__msg__RCRaw));
  bool success = rosflight_msgs__msg__RCRaw__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__msg__RCRaw__destroy(rosflight_msgs__msg__RCRaw * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__msg__RCRaw__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__msg__RCRaw__Sequence__init(rosflight_msgs__msg__RCRaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__RCRaw * data = NULL;

  if (size) {
    data = (rosflight_msgs__msg__RCRaw *)allocator.zero_allocate(size, sizeof(rosflight_msgs__msg__RCRaw), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__msg__RCRaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__msg__RCRaw__fini(&data[i - 1]);
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
rosflight_msgs__msg__RCRaw__Sequence__fini(rosflight_msgs__msg__RCRaw__Sequence * array)
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
      rosflight_msgs__msg__RCRaw__fini(&array->data[i]);
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

rosflight_msgs__msg__RCRaw__Sequence *
rosflight_msgs__msg__RCRaw__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__RCRaw__Sequence * array = (rosflight_msgs__msg__RCRaw__Sequence *)allocator.allocate(sizeof(rosflight_msgs__msg__RCRaw__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__msg__RCRaw__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__msg__RCRaw__Sequence__destroy(rosflight_msgs__msg__RCRaw__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__msg__RCRaw__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__msg__RCRaw__Sequence__are_equal(const rosflight_msgs__msg__RCRaw__Sequence * lhs, const rosflight_msgs__msg__RCRaw__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__msg__RCRaw__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__RCRaw__Sequence__copy(
  const rosflight_msgs__msg__RCRaw__Sequence * input,
  rosflight_msgs__msg__RCRaw__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__msg__RCRaw);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__msg__RCRaw * data =
      (rosflight_msgs__msg__RCRaw *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__msg__RCRaw__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__msg__RCRaw__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__msg__RCRaw__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
