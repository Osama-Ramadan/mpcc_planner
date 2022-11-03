// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice
#include "mpcc_interfaces/msg/detail/local_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `waypoints_x`
// Member `waypoints_y`
// Member `qx`
// Member `qy`
// Member `sampled_pt_x`
// Member `sampled_pt_y`
// Member `sampled_pt_th`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mpcc_interfaces__msg__LocalPath__init(mpcc_interfaces__msg__LocalPath * msg)
{
  if (!msg) {
    return false;
  }
  // waypoints_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->waypoints_x, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // waypoints_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->waypoints_y, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // qx
  if (!rosidl_runtime_c__float__Sequence__init(&msg->qx, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // qy
  if (!rosidl_runtime_c__float__Sequence__init(&msg->qy, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // sampled_pt_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->sampled_pt_x, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // sampled_pt_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->sampled_pt_y, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // sampled_pt_th
  if (!rosidl_runtime_c__float__Sequence__init(&msg->sampled_pt_th, 0)) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
    return false;
  }
  // nseg
  // resolution
  return true;
}

void
mpcc_interfaces__msg__LocalPath__fini(mpcc_interfaces__msg__LocalPath * msg)
{
  if (!msg) {
    return;
  }
  // waypoints_x
  rosidl_runtime_c__float__Sequence__fini(&msg->waypoints_x);
  // waypoints_y
  rosidl_runtime_c__float__Sequence__fini(&msg->waypoints_y);
  // qx
  rosidl_runtime_c__float__Sequence__fini(&msg->qx);
  // qy
  rosidl_runtime_c__float__Sequence__fini(&msg->qy);
  // sampled_pt_x
  rosidl_runtime_c__float__Sequence__fini(&msg->sampled_pt_x);
  // sampled_pt_y
  rosidl_runtime_c__float__Sequence__fini(&msg->sampled_pt_y);
  // sampled_pt_th
  rosidl_runtime_c__float__Sequence__fini(&msg->sampled_pt_th);
  // nseg
  // resolution
}

bool
mpcc_interfaces__msg__LocalPath__are_equal(const mpcc_interfaces__msg__LocalPath * lhs, const mpcc_interfaces__msg__LocalPath * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // waypoints_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->waypoints_x), &(rhs->waypoints_x)))
  {
    return false;
  }
  // waypoints_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->waypoints_y), &(rhs->waypoints_y)))
  {
    return false;
  }
  // qx
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->qx), &(rhs->qx)))
  {
    return false;
  }
  // qy
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->qy), &(rhs->qy)))
  {
    return false;
  }
  // sampled_pt_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->sampled_pt_x), &(rhs->sampled_pt_x)))
  {
    return false;
  }
  // sampled_pt_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->sampled_pt_y), &(rhs->sampled_pt_y)))
  {
    return false;
  }
  // sampled_pt_th
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->sampled_pt_th), &(rhs->sampled_pt_th)))
  {
    return false;
  }
  // nseg
  if (lhs->nseg != rhs->nseg) {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  return true;
}

bool
mpcc_interfaces__msg__LocalPath__copy(
  const mpcc_interfaces__msg__LocalPath * input,
  mpcc_interfaces__msg__LocalPath * output)
{
  if (!input || !output) {
    return false;
  }
  // waypoints_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->waypoints_x), &(output->waypoints_x)))
  {
    return false;
  }
  // waypoints_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->waypoints_y), &(output->waypoints_y)))
  {
    return false;
  }
  // qx
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->qx), &(output->qx)))
  {
    return false;
  }
  // qy
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->qy), &(output->qy)))
  {
    return false;
  }
  // sampled_pt_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->sampled_pt_x), &(output->sampled_pt_x)))
  {
    return false;
  }
  // sampled_pt_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->sampled_pt_y), &(output->sampled_pt_y)))
  {
    return false;
  }
  // sampled_pt_th
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->sampled_pt_th), &(output->sampled_pt_th)))
  {
    return false;
  }
  // nseg
  output->nseg = input->nseg;
  // resolution
  output->resolution = input->resolution;
  return true;
}

mpcc_interfaces__msg__LocalPath *
mpcc_interfaces__msg__LocalPath__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__LocalPath * msg = (mpcc_interfaces__msg__LocalPath *)allocator.allocate(sizeof(mpcc_interfaces__msg__LocalPath), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mpcc_interfaces__msg__LocalPath));
  bool success = mpcc_interfaces__msg__LocalPath__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mpcc_interfaces__msg__LocalPath__destroy(mpcc_interfaces__msg__LocalPath * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mpcc_interfaces__msg__LocalPath__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mpcc_interfaces__msg__LocalPath__Sequence__init(mpcc_interfaces__msg__LocalPath__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__LocalPath * data = NULL;

  if (size) {
    data = (mpcc_interfaces__msg__LocalPath *)allocator.zero_allocate(size, sizeof(mpcc_interfaces__msg__LocalPath), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mpcc_interfaces__msg__LocalPath__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mpcc_interfaces__msg__LocalPath__fini(&data[i - 1]);
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
mpcc_interfaces__msg__LocalPath__Sequence__fini(mpcc_interfaces__msg__LocalPath__Sequence * array)
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
      mpcc_interfaces__msg__LocalPath__fini(&array->data[i]);
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

mpcc_interfaces__msg__LocalPath__Sequence *
mpcc_interfaces__msg__LocalPath__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mpcc_interfaces__msg__LocalPath__Sequence * array = (mpcc_interfaces__msg__LocalPath__Sequence *)allocator.allocate(sizeof(mpcc_interfaces__msg__LocalPath__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mpcc_interfaces__msg__LocalPath__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mpcc_interfaces__msg__LocalPath__Sequence__destroy(mpcc_interfaces__msg__LocalPath__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mpcc_interfaces__msg__LocalPath__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mpcc_interfaces__msg__LocalPath__Sequence__are_equal(const mpcc_interfaces__msg__LocalPath__Sequence * lhs, const mpcc_interfaces__msg__LocalPath__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mpcc_interfaces__msg__LocalPath__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mpcc_interfaces__msg__LocalPath__Sequence__copy(
  const mpcc_interfaces__msg__LocalPath__Sequence * input,
  mpcc_interfaces__msg__LocalPath__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mpcc_interfaces__msg__LocalPath);
    mpcc_interfaces__msg__LocalPath * data =
      (mpcc_interfaces__msg__LocalPath *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mpcc_interfaces__msg__LocalPath__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          mpcc_interfaces__msg__LocalPath__fini(&data[i]);
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
    if (!mpcc_interfaces__msg__LocalPath__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
