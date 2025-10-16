// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:msg/GNSS.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/msg/detail/gnss__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rosflight_msgs__msg__GNSS__init(rosflight_msgs__msg__GNSS * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rosflight_msgs__msg__GNSS__fini(msg);
    return false;
  }
  // fix_type
  // num_sat
  // lat
  // lon
  // alt
  // horizontal_accuracy
  // vertical_accuracy
  // vel_n
  // vel_e
  // vel_d
  // speed_accuracy
  // gnss_unix_seconds
  // gnss_unix_nanos
  return true;
}

void
rosflight_msgs__msg__GNSS__fini(rosflight_msgs__msg__GNSS * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fix_type
  // num_sat
  // lat
  // lon
  // alt
  // horizontal_accuracy
  // vertical_accuracy
  // vel_n
  // vel_e
  // vel_d
  // speed_accuracy
  // gnss_unix_seconds
  // gnss_unix_nanos
}

bool
rosflight_msgs__msg__GNSS__are_equal(const rosflight_msgs__msg__GNSS * lhs, const rosflight_msgs__msg__GNSS * rhs)
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
  // fix_type
  if (lhs->fix_type != rhs->fix_type) {
    return false;
  }
  // num_sat
  if (lhs->num_sat != rhs->num_sat) {
    return false;
  }
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // alt
  if (lhs->alt != rhs->alt) {
    return false;
  }
  // horizontal_accuracy
  if (lhs->horizontal_accuracy != rhs->horizontal_accuracy) {
    return false;
  }
  // vertical_accuracy
  if (lhs->vertical_accuracy != rhs->vertical_accuracy) {
    return false;
  }
  // vel_n
  if (lhs->vel_n != rhs->vel_n) {
    return false;
  }
  // vel_e
  if (lhs->vel_e != rhs->vel_e) {
    return false;
  }
  // vel_d
  if (lhs->vel_d != rhs->vel_d) {
    return false;
  }
  // speed_accuracy
  if (lhs->speed_accuracy != rhs->speed_accuracy) {
    return false;
  }
  // gnss_unix_seconds
  if (lhs->gnss_unix_seconds != rhs->gnss_unix_seconds) {
    return false;
  }
  // gnss_unix_nanos
  if (lhs->gnss_unix_nanos != rhs->gnss_unix_nanos) {
    return false;
  }
  return true;
}

bool
rosflight_msgs__msg__GNSS__copy(
  const rosflight_msgs__msg__GNSS * input,
  rosflight_msgs__msg__GNSS * output)
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
  // fix_type
  output->fix_type = input->fix_type;
  // num_sat
  output->num_sat = input->num_sat;
  // lat
  output->lat = input->lat;
  // lon
  output->lon = input->lon;
  // alt
  output->alt = input->alt;
  // horizontal_accuracy
  output->horizontal_accuracy = input->horizontal_accuracy;
  // vertical_accuracy
  output->vertical_accuracy = input->vertical_accuracy;
  // vel_n
  output->vel_n = input->vel_n;
  // vel_e
  output->vel_e = input->vel_e;
  // vel_d
  output->vel_d = input->vel_d;
  // speed_accuracy
  output->speed_accuracy = input->speed_accuracy;
  // gnss_unix_seconds
  output->gnss_unix_seconds = input->gnss_unix_seconds;
  // gnss_unix_nanos
  output->gnss_unix_nanos = input->gnss_unix_nanos;
  return true;
}

rosflight_msgs__msg__GNSS *
rosflight_msgs__msg__GNSS__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__GNSS * msg = (rosflight_msgs__msg__GNSS *)allocator.allocate(sizeof(rosflight_msgs__msg__GNSS), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__msg__GNSS));
  bool success = rosflight_msgs__msg__GNSS__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__msg__GNSS__destroy(rosflight_msgs__msg__GNSS * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__msg__GNSS__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__msg__GNSS__Sequence__init(rosflight_msgs__msg__GNSS__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__GNSS * data = NULL;

  if (size) {
    data = (rosflight_msgs__msg__GNSS *)allocator.zero_allocate(size, sizeof(rosflight_msgs__msg__GNSS), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__msg__GNSS__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__msg__GNSS__fini(&data[i - 1]);
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
rosflight_msgs__msg__GNSS__Sequence__fini(rosflight_msgs__msg__GNSS__Sequence * array)
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
      rosflight_msgs__msg__GNSS__fini(&array->data[i]);
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

rosflight_msgs__msg__GNSS__Sequence *
rosflight_msgs__msg__GNSS__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__msg__GNSS__Sequence * array = (rosflight_msgs__msg__GNSS__Sequence *)allocator.allocate(sizeof(rosflight_msgs__msg__GNSS__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__msg__GNSS__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__msg__GNSS__Sequence__destroy(rosflight_msgs__msg__GNSS__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__msg__GNSS__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__msg__GNSS__Sequence__are_equal(const rosflight_msgs__msg__GNSS__Sequence * lhs, const rosflight_msgs__msg__GNSS__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__msg__GNSS__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__msg__GNSS__Sequence__copy(
  const rosflight_msgs__msg__GNSS__Sequence * input,
  rosflight_msgs__msg__GNSS__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__msg__GNSS);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__msg__GNSS * data =
      (rosflight_msgs__msg__GNSS *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__msg__GNSS__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__msg__GNSS__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__msg__GNSS__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
