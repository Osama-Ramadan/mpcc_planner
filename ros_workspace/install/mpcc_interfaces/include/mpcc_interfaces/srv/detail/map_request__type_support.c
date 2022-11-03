// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mpcc_interfaces/srv/detail/map_request__rosidl_typesupport_introspection_c.h"
#include "mpcc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mpcc_interfaces/srv/detail/map_request__functions.h"
#include "mpcc_interfaces/srv/detail/map_request__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mpcc_interfaces__srv__MapRequest_Request__init(message_memory);
}

void MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_fini_function(void * message_memory)
{
  mpcc_interfaces__srv__MapRequest_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_member_array[4] = {
  {
    "x_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__srv__MapRequest_Request, x_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__srv__MapRequest_Request, x_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__srv__MapRequest_Request, y_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__srv__MapRequest_Request, y_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_members = {
  "mpcc_interfaces__srv",  // message namespace
  "MapRequest_Request",  // message name
  4,  // number of fields
  sizeof(mpcc_interfaces__srv__MapRequest_Request),
  MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_member_array,  // message members
  MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_type_support_handle = {
  0,
  &MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mpcc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Request)() {
  if (!MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_type_support_handle.typesupport_identifier) {
    MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MapRequest_Request__rosidl_typesupport_introspection_c__MapRequest_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mpcc_interfaces/srv/detail/map_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mpcc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mpcc_interfaces/srv/detail/map_request__functions.h"
// already included above
// #include "mpcc_interfaces/srv/detail/map_request__struct.h"


// Include directives for member types
// Member `costmap`
#include "nav2_msgs/msg/costmap.h"
// Member `costmap`
#include "nav2_msgs/msg/detail/costmap__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mpcc_interfaces__srv__MapRequest_Response__init(message_memory);
}

void MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_fini_function(void * message_memory)
{
  mpcc_interfaces__srv__MapRequest_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_member_array[1] = {
  {
    "costmap",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces__srv__MapRequest_Response, costmap),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_members = {
  "mpcc_interfaces__srv",  // message namespace
  "MapRequest_Response",  // message name
  1,  // number of fields
  sizeof(mpcc_interfaces__srv__MapRequest_Response),
  MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_member_array,  // message members
  MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_type_support_handle = {
  0,
  &MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mpcc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Response)() {
  MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav2_msgs, msg, Costmap)();
  if (!MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_type_support_handle.typesupport_identifier) {
    MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MapRequest_Response__rosidl_typesupport_introspection_c__MapRequest_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mpcc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mpcc_interfaces/srv/detail/map_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_members = {
  "mpcc_interfaces__srv",  // service namespace
  "MapRequest",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_Request_message_type_support_handle,
  NULL  // response message
  // mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_Response_message_type_support_handle
};

static rosidl_service_type_support_t mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_type_support_handle = {
  0,
  &mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mpcc_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest)() {
  if (!mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_type_support_handle.typesupport_identifier) {
    mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mpcc_interfaces, srv, MapRequest_Response)()->data;
  }

  return &mpcc_interfaces__srv__detail__map_request__rosidl_typesupport_introspection_c__MapRequest_service_type_support_handle;
}
