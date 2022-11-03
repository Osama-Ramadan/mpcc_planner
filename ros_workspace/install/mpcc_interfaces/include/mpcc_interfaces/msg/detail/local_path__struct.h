// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_H_
#define MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'waypoints_x'
// Member 'waypoints_y'
// Member 'qx'
// Member 'qy'
// Member 'sampled_pt_x'
// Member 'sampled_pt_y'
// Member 'sampled_pt_th'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/LocalPath in the package mpcc_interfaces.
typedef struct mpcc_interfaces__msg__LocalPath
{
  rosidl_runtime_c__float__Sequence waypoints_x;
  rosidl_runtime_c__float__Sequence waypoints_y;
  rosidl_runtime_c__float__Sequence qx;
  rosidl_runtime_c__float__Sequence qy;
  rosidl_runtime_c__float__Sequence sampled_pt_x;
  rosidl_runtime_c__float__Sequence sampled_pt_y;
  rosidl_runtime_c__float__Sequence sampled_pt_th;
  int32_t nseg;
  int32_t resolution;
} mpcc_interfaces__msg__LocalPath;

// Struct for a sequence of mpcc_interfaces__msg__LocalPath.
typedef struct mpcc_interfaces__msg__LocalPath__Sequence
{
  mpcc_interfaces__msg__LocalPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mpcc_interfaces__msg__LocalPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_H_
