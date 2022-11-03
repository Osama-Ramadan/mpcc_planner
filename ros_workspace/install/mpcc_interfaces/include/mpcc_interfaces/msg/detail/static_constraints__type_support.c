// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mpcc_interfaces/msg/detail/static_constraints__rosidl_typesupport_introspection_c.h"
#include "mpcc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mpcc_interfaces/msg/detail/static_constraints__functions.h"
#include "mpcc_interfaces/msg/detail/static_constraints__struct.h"


// Include directives for member types
// Member `point_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `ul_x`
// Member `ul_y`
// Member `lr_x`
// Member `lr_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mpcc_interfaces__msg__StaticConstraints__init(message_memory);
}

void StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_fini_function(void * message_memory)
{
  mpcc_interfaces__msg__StaticConstraints__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_member_array[5] = {
  {
    "point_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__StaticConstraints, point_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ul_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__StaticConstraints, ul_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ul_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__StaticConstraints, ul_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lr_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__StaticConstraints, lr_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lr_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__StaticConstraints, lr_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_members = {
  "mpcc_interfaces__msg",  // message namespace
  "StaticConstraints",  // message name
  5,  // number of fields
  sizeof(mpcc_interfaces__msg__StaticConstraints),
  StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_member_array,  // message members
  StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_init_function,  // function to initialize message memory (memory has to be allocated)
  StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_type_support_handle = {
  0,
  &StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mpcc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, msg, StaticConstraints)() {
  if (!StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_type_support_handle.typesupport_identifier) {
    StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StaticConstraints__rosidl_typesupport_introspection_c__StaticConstraints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
