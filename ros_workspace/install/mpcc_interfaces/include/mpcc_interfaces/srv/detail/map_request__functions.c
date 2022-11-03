// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice
#include "mpcc_interfaces/srv/detail/map_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mpcc_interfaces__srv__MapRequest_Request__init(mpcc_interfaces__srv__MapRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x_min
  // x_max
  // y_min
  // y_max
  return true;
}

void
mpcc_interfaces__srv__MapRequest_Request__fini(mpcc_interfaces__srv__MapRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // x_min
  // x_max
  // y_min
  // y_max
}

bool
mpcc_interfaces__srv__MapRequest_Request__are_equal(const mpcc_interfaces__srv__MapRequest_Request * lhs, const mpcc_interfaces__srv__MapRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_min
  if (lhs->x_min != rhs->x_min) {
    return false;
  }
  // x_max
  if (lhs->x_max != rhs->x_max) {
    return false;
  }
  // y_min
  if (lhs->y_min != rhs->y_min) {
    return false;
  }
  // y_max
  if (lhs->y_max != rhs->y_max) {
    return false;
  }
  return true;
}

bool
mpcc_interfaces__srv__MapRequest_Request__copy(
  const mpcc_interfaces__srv__MapRequest_Request * input,
  mpcc_interfaces__srv__MapRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x_min
  output->x_min = input->x_min;
  // x_max
  output->x_max = input->x_max;
  // y_min
  output->y_min = input->y_min;
  // y_max
  output->y_max = input->y_max;
  return true;
}

mpcc_interfaces__srv__MapRequest_Request *
mpcc_interfaces__srv__MapRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Request * msg = (mpcc_interfaces__srv__MapRequest_Request *)allocator.allocate(sizeof(mpcc_interfaces__srv__MapRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mpcc_interfaces__srv__MapRequest_Request));
  bool success = mpcc_interfaces__srv__MapRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mpcc_interfaces__srv__MapRequest_Request__destroy(mpcc_interfaces__srv__MapRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mpcc_interfaces__srv__MapRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__init(mpcc_interfaces__srv__MapRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Request * data = NULL;

  if (size) {
    data = (mpcc_interfaces__srv__MapRequest_Request *)allocator.zero_allocate(size, sizeof(mpcc_interfaces__srv__MapRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mpcc_interfaces__srv__MapRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mpcc_interfaces__srv__MapRequest_Request__fini(&data[i - 1]);
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
mpcc_interfaces__srv__MapRequest_Request__Sequence__fini(mpcc_interfaces__srv__MapRequest_Request__Sequence * array)
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
      mpcc_interfaces__srv__MapRequest_Request__fini(&array->data[i]);
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

mpcc_interfaces__srv__MapRequest_Request__Sequence *
mpcc_interfaces__srv__MapRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Request__Sequence * array = (mpcc_interfaces__srv__MapRequest_Request__Sequence *)allocator.allocate(sizeof(mpcc_interfaces__srv__MapRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mpcc_interfaces__srv__MapRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mpcc_interfaces__srv__MapRequest_Request__Sequence__destroy(mpcc_interfaces__srv__MapRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mpcc_interfaces__srv__MapRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__are_equal(const mpcc_interfaces__srv__MapRequest_Request__Sequence * lhs, const mpcc_interfaces__srv__MapRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mpcc_interfaces__srv__MapRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__copy(
  const mpcc_interfaces__srv__MapRequest_Request__Sequence * input,
  mpcc_interfaces__srv__MapRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mpcc_interfaces__srv__MapRequest_Request);
    mpcc_interfaces__srv__MapRequest_Request * data =
      (mpcc_interfaces__srv__MapRequest_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mpcc_interfaces__srv__MapRequest_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mpcc_interfaces__srv__MapRequest_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mpcc_interfaces__srv__MapRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `costmap`
#include "nav2_msgs/msg/detail/costmap__functions.h"

bool
mpcc_interfaces__srv__MapRequest_Response__init(mpcc_interfaces__srv__MapRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // costmap
  if (!nav2_msgs__msg__Costmap__init(&msg->costmap)) {
    mpcc_interfaces__srv__MapRequest_Response__fini(msg);
    return false;
  }
  return true;
}

void
mpcc_interfaces__srv__MapRequest_Response__fini(mpcc_interfaces__srv__MapRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // costmap
  nav2_msgs__msg__Costmap__fini(&msg->costmap);
}

bool
mpcc_interfaces__srv__MapRequest_Response__are_equal(const mpcc_interfaces__srv__MapRequest_Response * lhs, const mpcc_interfaces__srv__MapRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // costmap
  if (!nav2_msgs__msg__Costmap__are_equal(
      &(lhs->costmap), &(rhs->costmap)))
  {
    return false;
  }
  return true;
}

bool
mpcc_interfaces__srv__MapRequest_Response__copy(
  const mpcc_interfaces__srv__MapRequest_Response * input,
  mpcc_interfaces__srv__MapRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // costmap
  if (!nav2_msgs__msg__Costmap__copy(
      &(input->costmap), &(output->costmap)))
  {
    return false;
  }
  return true;
}

mpcc_interfaces__srv__MapRequest_Response *
mpcc_interfaces__srv__MapRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Response * msg = (mpcc_interfaces__srv__MapRequest_Response *)allocator.allocate(sizeof(mpcc_interfaces__srv__MapRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mpcc_interfaces__srv__MapRequest_Response));
  bool success = mpcc_interfaces__srv__MapRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mpcc_interfaces__srv__MapRequest_Response__destroy(mpcc_interfaces__srv__MapRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mpcc_interfaces__srv__MapRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__init(mpcc_interfaces__srv__MapRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Response * data = NULL;

  if (size) {
    data = (mpcc_interfaces__srv__MapRequest_Response *)allocator.zero_allocate(size, sizeof(mpcc_interfaces__srv__MapRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mpcc_interfaces__srv__MapRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mpcc_interfaces__srv__MapRequest_Response__fini(&data[i - 1]);
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
mpcc_interfaces__srv__MapRequest_Response__Sequence__fini(mpcc_interfaces__srv__MapRequest_Response__Sequence * array)
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
      mpcc_interfaces__srv__MapRequest_Response__fini(&array->data[i]);
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

mpcc_interfaces__srv__MapRequest_Response__Sequence *
mpcc_interfaces__srv__MapRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__srv__MapRequest_Response__Sequence * array = (mpcc_interfaces__srv__MapRequest_Response__Sequence *)allocator.allocate(sizeof(mpcc_interfaces__srv__MapRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mpcc_interfaces__srv__MapRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mpcc_interfaces__srv__MapRequest_Response__Sequence__destroy(mpcc_interfaces__srv__MapRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mpcc_interfaces__srv__MapRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__are_equal(const mpcc_interfaces__srv__MapRequest_Response__Sequence * lhs, const mpcc_interfaces__srv__MapRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mpcc_interfaces__srv__MapRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__copy(
  const mpcc_interfaces__srv__MapRequest_Response__Sequence * input,
  mpcc_interfaces__srv__MapRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mpcc_interfaces__srv__MapRequest_Response);
    mpcc_interfaces__srv__MapRequest_Response * data =
      (mpcc_interfaces__srv__MapRequest_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mpcc_interfaces__srv__MapRequest_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mpcc_interfaces__srv__MapRequest_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mpcc_interfaces__srv__MapRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
