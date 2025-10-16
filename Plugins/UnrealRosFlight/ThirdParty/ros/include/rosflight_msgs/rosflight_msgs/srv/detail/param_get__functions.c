// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosflight_msgs:srv/ParamGet.idl
// generated code does not contain a copyright notice
#include "rosflight_msgs/srv/detail/param_get__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
rosflight_msgs__srv__ParamGet_Request__init(rosflight_msgs__srv__ParamGet_Request * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rosflight_msgs__srv__ParamGet_Request__fini(msg);
    return false;
  }
  return true;
}

void
rosflight_msgs__srv__ParamGet_Request__fini(rosflight_msgs__srv__ParamGet_Request * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
}

bool
rosflight_msgs__srv__ParamGet_Request__are_equal(const rosflight_msgs__srv__ParamGet_Request * lhs, const rosflight_msgs__srv__ParamGet_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  return true;
}

bool
rosflight_msgs__srv__ParamGet_Request__copy(
  const rosflight_msgs__srv__ParamGet_Request * input,
  rosflight_msgs__srv__ParamGet_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  return true;
}

rosflight_msgs__srv__ParamGet_Request *
rosflight_msgs__srv__ParamGet_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Request * msg = (rosflight_msgs__srv__ParamGet_Request *)allocator.allocate(sizeof(rosflight_msgs__srv__ParamGet_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__srv__ParamGet_Request));
  bool success = rosflight_msgs__srv__ParamGet_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__srv__ParamGet_Request__destroy(rosflight_msgs__srv__ParamGet_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__srv__ParamGet_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__srv__ParamGet_Request__Sequence__init(rosflight_msgs__srv__ParamGet_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Request * data = NULL;

  if (size) {
    data = (rosflight_msgs__srv__ParamGet_Request *)allocator.zero_allocate(size, sizeof(rosflight_msgs__srv__ParamGet_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__srv__ParamGet_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__srv__ParamGet_Request__fini(&data[i - 1]);
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
rosflight_msgs__srv__ParamGet_Request__Sequence__fini(rosflight_msgs__srv__ParamGet_Request__Sequence * array)
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
      rosflight_msgs__srv__ParamGet_Request__fini(&array->data[i]);
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

rosflight_msgs__srv__ParamGet_Request__Sequence *
rosflight_msgs__srv__ParamGet_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Request__Sequence * array = (rosflight_msgs__srv__ParamGet_Request__Sequence *)allocator.allocate(sizeof(rosflight_msgs__srv__ParamGet_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__srv__ParamGet_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__srv__ParamGet_Request__Sequence__destroy(rosflight_msgs__srv__ParamGet_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__srv__ParamGet_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__srv__ParamGet_Request__Sequence__are_equal(const rosflight_msgs__srv__ParamGet_Request__Sequence * lhs, const rosflight_msgs__srv__ParamGet_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__srv__ParamGet_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__srv__ParamGet_Request__Sequence__copy(
  const rosflight_msgs__srv__ParamGet_Request__Sequence * input,
  rosflight_msgs__srv__ParamGet_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__srv__ParamGet_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__srv__ParamGet_Request * data =
      (rosflight_msgs__srv__ParamGet_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__srv__ParamGet_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__srv__ParamGet_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__srv__ParamGet_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rosflight_msgs__srv__ParamGet_Response__init(rosflight_msgs__srv__ParamGet_Response * msg)
{
  if (!msg) {
    return false;
  }
  // exists
  // value
  return true;
}

void
rosflight_msgs__srv__ParamGet_Response__fini(rosflight_msgs__srv__ParamGet_Response * msg)
{
  if (!msg) {
    return;
  }
  // exists
  // value
}

bool
rosflight_msgs__srv__ParamGet_Response__are_equal(const rosflight_msgs__srv__ParamGet_Response * lhs, const rosflight_msgs__srv__ParamGet_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // exists
  if (lhs->exists != rhs->exists) {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  return true;
}

bool
rosflight_msgs__srv__ParamGet_Response__copy(
  const rosflight_msgs__srv__ParamGet_Response * input,
  rosflight_msgs__srv__ParamGet_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // exists
  output->exists = input->exists;
  // value
  output->value = input->value;
  return true;
}

rosflight_msgs__srv__ParamGet_Response *
rosflight_msgs__srv__ParamGet_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Response * msg = (rosflight_msgs__srv__ParamGet_Response *)allocator.allocate(sizeof(rosflight_msgs__srv__ParamGet_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosflight_msgs__srv__ParamGet_Response));
  bool success = rosflight_msgs__srv__ParamGet_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosflight_msgs__srv__ParamGet_Response__destroy(rosflight_msgs__srv__ParamGet_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosflight_msgs__srv__ParamGet_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosflight_msgs__srv__ParamGet_Response__Sequence__init(rosflight_msgs__srv__ParamGet_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Response * data = NULL;

  if (size) {
    data = (rosflight_msgs__srv__ParamGet_Response *)allocator.zero_allocate(size, sizeof(rosflight_msgs__srv__ParamGet_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosflight_msgs__srv__ParamGet_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosflight_msgs__srv__ParamGet_Response__fini(&data[i - 1]);
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
rosflight_msgs__srv__ParamGet_Response__Sequence__fini(rosflight_msgs__srv__ParamGet_Response__Sequence * array)
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
      rosflight_msgs__srv__ParamGet_Response__fini(&array->data[i]);
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

rosflight_msgs__srv__ParamGet_Response__Sequence *
rosflight_msgs__srv__ParamGet_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosflight_msgs__srv__ParamGet_Response__Sequence * array = (rosflight_msgs__srv__ParamGet_Response__Sequence *)allocator.allocate(sizeof(rosflight_msgs__srv__ParamGet_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosflight_msgs__srv__ParamGet_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosflight_msgs__srv__ParamGet_Response__Sequence__destroy(rosflight_msgs__srv__ParamGet_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosflight_msgs__srv__ParamGet_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosflight_msgs__srv__ParamGet_Response__Sequence__are_equal(const rosflight_msgs__srv__ParamGet_Response__Sequence * lhs, const rosflight_msgs__srv__ParamGet_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosflight_msgs__srv__ParamGet_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosflight_msgs__srv__ParamGet_Response__Sequence__copy(
  const rosflight_msgs__srv__ParamGet_Response__Sequence * input,
  rosflight_msgs__srv__ParamGet_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosflight_msgs__srv__ParamGet_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosflight_msgs__srv__ParamGet_Response * data =
      (rosflight_msgs__srv__ParamGet_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosflight_msgs__srv__ParamGet_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosflight_msgs__srv__ParamGet_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosflight_msgs__srv__ParamGet_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
