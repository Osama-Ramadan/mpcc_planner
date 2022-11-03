// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mpcc_interfaces/msg/detail/local_path__rosidl_typesupport_introspection_c.h"
#include "mpcc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mpcc_interfaces/msg/detail/local_path__functions.h"
#include "mpcc_interfaces/msg/detail/local_path__struct.h"


// Include directives for member types
// Member `waypoints_x`
// Member `waypoints_y`
// Member `qx`
// Member `qy`
// Member `sampled_pt_x`
// Member `sampled_pt_y`
// Member `sampled_pt_th`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void LocalPath__rosidl_typesupport_introspection_c__LocalPath_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mpcc_interfaces__msg__LocalPath__init(message_memory);
}

void LocalPath__rosidl_typesupport_introspection_c__LocalPath_fini_function(void * message_memory)
{
  mpcc_interfaces__msg__LocalPath__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_member_array[9] = {
  {
    "waypoints_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, waypoints_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, waypoints_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "qx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, qx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "qy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, qy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sampled_pt_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, sampled_pt_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sampled_pt_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, sampled_pt_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sampled_pt_th",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, sampled_pt_th),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nseg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, nseg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resolution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__msg__LocalPath, resolution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_members = {
  "mpcc_interfaces__msg",  // message namespace
  "LocalPath",  // message name
  9,  // number of fields
  sizeof(mpcc_interfaces__msg__LocalPath),
  LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_member_array,  // message members
  LocalPath__rosidl_typesupport_introspection_c__LocalPath_init_function,  // function to initialize message memory (memory has to be allocated)
  LocalPath__rosidl_typesupport_introspection_c__LocalPath_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_type_support_handle = {
  0,
  &LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mpcc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, msg, LocalPath)() {
  if (!LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_type_support_handle.typesupport_identifier) {
    LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LocalPath__rosidl_typesupport_introspection_c__LocalPath_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
