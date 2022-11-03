// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__STRUCT_H_
#define MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/MapRequest in the package mpcc_interfaces.
typedef struct mpcc_interfaces__srv__MapRequest_Request
{
  float x_min;
  float x_max;
  float y_min;
  float y_max;
} mpcc_interfaces__srv__MapRequest_Request;

// Struct for a sequence of mpcc_interfaces__srv__MapRequest_Request.
typedef struct mpcc_interfaces__srv__MapRequest_Request__Sequence
{
  mpcc_interfaces__srv__MapRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mpcc_interfaces__srv__MapRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'costmap'
#include "nav2_msgs/msg/detail/costmap__struct.h"

// Struct defined in srv/MapRequest in the package mpcc_interfaces.
typedef struct mpcc_interfaces__srv__MapRequest_Response
{
  nav2_msgs__msg__Costmap costmap;
} mpcc_interfaces__srv__MapRequest_Response;

// Struct for a sequence of mpcc_interfaces__srv__MapRequest_Response.
typedef struct mpcc_interfaces__srv__MapRequest_Response__Sequence
{
  mpcc_interfaces__srv__MapRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mpcc_interfaces__srv__MapRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__STRUCT_H_
