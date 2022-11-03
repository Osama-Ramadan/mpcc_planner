// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice
#include "mpcc_interfaces/msg/detail/static_constraints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `point_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `ul_x`
// Member `ul_y`
// Member `lr_x`
// Member `lr_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mpcc_interfaces__msg__StaticConstraints__init(mpcc_interfaces__msg__StaticConstraints * msg)
{
  if (!msg) {
    return false;
  }
  // point_type
  if (!rosidl_runtime_c__String__init(&msg->point_type)) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
    return false;
  }
  // ul_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ul_x, 0)) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
    return false;
  }
  // ul_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->ul_y, 0)) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
    return false;
  }
  // lr_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->lr_x, 0)) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
    return false;
  }
  // lr_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->lr_y, 0)) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
    return false;
  }
  return true;
}

void
mpcc_interfaces__msg__StaticConstraints__fini(mpcc_interfaces__msg__StaticConstraints * msg)
{
  if (!msg) {
    return;
  }
  // point_type
  rosidl_runtime_c__String__fini(&msg->point_type);
  // ul_x
  rosidl_runtime_c__float__Sequence__fini(&msg->ul_x);
  // ul_y
  rosidl_runtime_c__float__Sequence__fini(&msg->ul_y);
  // lr_x
  rosidl_runtime_c__float__Sequence__fini(&msg->lr_x);
  // lr_y
  rosidl_runtime_c__float__Sequence__fini(&msg->lr_y);
}

bool
mpcc_interfaces__msg__StaticConstraints__are_equal(const mpcc_interfaces__msg__StaticConstraints * lhs, const mpcc_interfaces__msg__StaticConstraints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // point_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->point_type), &(rhs->point_type)))
  {
    return false;
  }
  // ul_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ul_x), &(rhs->ul_x)))
  {
    return false;
  }
  // ul_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->ul_y), &(rhs->ul_y)))
  {
    return false;
  }
  // lr_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->lr_x), &(rhs->lr_x)))
  {
    return false;
  }
  // lr_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->lr_y), &(rhs->lr_y)))
  {
    return false;
  }
  return true;
}

bool
mpcc_interfaces__msg__StaticConstraints__copy(
  const mpcc_interfaces__msg__StaticConstraints * input,
  mpcc_interfaces__msg__StaticConstraints * output)
{
  if (!input || !output) {
    return false;
  }
  // point_type
  if (!rosidl_runtime_c__String__copy(
      &(input->point_type), &(output->point_type)))
  {
    return false;
  }
  // ul_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ul_x), &(output->ul_x)))
  {
    return false;
  }
  // ul_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->ul_y), &(output->ul_y)))
  {
    return false;
  }
  // lr_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->lr_x), &(output->lr_x)))
  {
    return false;
  }
  // lr_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->lr_y), &(output->lr_y)))
  {
    return false;
  }
  return true;
}

mpcc_interfaces__msg__StaticConstraints *
mpcc_interfaces__msg__StaticConstraints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__StaticConstraints * msg = (mpcc_interfaces__msg__StaticConstraints *)allocator.allocate(sizeof(mpcc_interfaces__msg__StaticConstraints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mpcc_interfaces__msg__StaticConstraints));
  bool success = mpcc_interfaces__msg__StaticConstraints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mpcc_interfaces__msg__StaticConstraints__destroy(mpcc_interfaces__msg__StaticConstraints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mpcc_interfaces__msg__StaticConstraints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mpcc_interfaces__msg__StaticConstraints__Sequence__init(mpcc_interfaces__msg__StaticConstraints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__StaticConstraints * data = NULL;

  if (size) {
    data = (mpcc_interfaces__msg__StaticConstraints *)allocator.zero_allocate(size, sizeof(mpcc_interfaces__msg__StaticConstraints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mpcc_interfaces__msg__StaticConstraints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mpcc_interfaces__msg__StaticConstraints__fini(&data[i - 1]);
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
mpcc_interfaces__msg__StaticConstraints__Sequence__fini(mpcc_interfaces__msg__StaticConstraints__Sequence * array)
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
      mpcc_interfaces__msg__StaticConstraints__fini(&array->data[i]);
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

mpcc_interfaces__msg__StaticConstraints__Sequence *
mpcc_interfaces__msg__StaticConstraints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__StaticConstraints__Sequence * array = (mpcc_interfaces__msg__StaticConstraints__Sequence *)allocator.allocate(sizeof(mpcc_interfaces__msg__StaticConstraints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mpcc_interfaces__msg__StaticConstraints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mpcc_interfaces__msg__StaticConstraints__Sequence__destroy(mpcc_interfaces__msg__StaticConstraints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mpcc_interfaces__msg__StaticConstraints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mpcc_interfaces__msg__StaticConstraints__Sequence__are_equal(const mpcc_interfaces__msg__StaticConstraints__Sequence * lhs, const mpcc_interfaces__msg__StaticConstraints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mpcc_interfaces__msg__StaticConstraints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mpcc_interfaces__msg__StaticConstraints__Sequence__copy(
  const mpcc_interfaces__msg__StaticConstraints__Sequence * input,
  mpcc_interfaces__msg__StaticConstraints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mpcc_interfaces__msg__StaticConstraints);
    mpcc_interfaces__msg__StaticConstraints * data =
      (mpcc_interfaces__msg__StaticConstraints *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mpcc_interfaces__msg__StaticConstraints__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mpcc_interfaces__msg__StaticConstraints__fini(&data[i]);
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
    if (!mpcc_interfaces__msg__StaticConstraints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
