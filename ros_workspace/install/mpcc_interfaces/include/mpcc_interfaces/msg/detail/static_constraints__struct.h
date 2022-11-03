// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_H_
#define MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point_type'
#include "rosidl_runtime_c/string.h"
// Member 'ul_x'
// Member 'ul_y'
// Member 'lr_x'
// Member 'lr_y'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/StaticConstraints in the package mpcc_interfaces.
typedef struct mpcc_interfaces__msg__StaticConstraints
{
  rosidl_runtime_c__String point_type;
  rosidl_runtime_c__float__Sequence ul_x;
  rosidl_runtime_c__float__Sequence ul_y;
  rosidl_runtime_c__float__Sequence lr_x;
  rosidl_runtime_c__float__Sequence lr_y;
} mpcc_interfaces__msg__StaticConstraints;

// Struct for a sequence of mpcc_interfaces__msg__StaticConstraints.
typedef struct mpcc_interfaces__msg__StaticConstraints__Sequence
{
  mpcc_interfaces__msg__StaticConstraints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mpcc_interfaces__msg__StaticConstraints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_H_
