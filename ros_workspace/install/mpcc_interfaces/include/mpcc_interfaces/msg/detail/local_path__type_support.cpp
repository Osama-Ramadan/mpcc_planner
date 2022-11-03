// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mpcc_interfaces/msg/detail/local_path__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mpcc_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LocalPath_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mpcc_interfaces::msg::LocalPath(_init);
}

void LocalPath_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mpcc_interfaces::msg::LocalPath *>(message_memory);
  typed_message->~LocalPath();
}

size_t size_function__LocalPath__waypoints_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__waypoints_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__waypoints_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__waypoints_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__waypoints_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__waypoints_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__waypoints_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__waypoints_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__qx(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__qx(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__qx(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__qx(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__qy(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__qy(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__qy(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__qy(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__sampled_pt_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__sampled_pt_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__sampled_pt_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__sampled_pt_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__sampled_pt_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__sampled_pt_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__sampled_pt_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__sampled_pt_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LocalPath__sampled_pt_th(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPath__sampled_pt_th(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPath__sampled_pt_th(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__LocalPath__sampled_pt_th(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LocalPath_message_member_array[9] = {
  {
    "waypoints_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, waypoints_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__waypoints_x,  // size() function pointer
    get_const_function__LocalPath__waypoints_x,  // get_const(index) function pointer
    get_function__LocalPath__waypoints_x,  // get(index) function pointer
    resize_function__LocalPath__waypoints_x  // resize(index) function pointer
  },
  {
    "waypoints_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, waypoints_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__waypoints_y,  // size() function pointer
    get_const_function__LocalPath__waypoints_y,  // get_const(index) function pointer
    get_function__LocalPath__waypoints_y,  // get(index) function pointer
    resize_function__LocalPath__waypoints_y  // resize(index) function pointer
  },
  {
    "qx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, qx),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__qx,  // size() function pointer
    get_const_function__LocalPath__qx,  // get_const(index) function pointer
    get_function__LocalPath__qx,  // get(index) function pointer
    resize_function__LocalPath__qx  // resize(index) function pointer
  },
  {
    "qy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, qy),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__qy,  // size() function pointer
    get_const_function__LocalPath__qy,  // get_const(index) function pointer
    get_function__LocalPath__qy,  // get(index) function pointer
    resize_function__LocalPath__qy  // resize(index) function pointer
  },
  {
    "sampled_pt_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, sampled_pt_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__sampled_pt_x,  // size() function pointer
    get_const_function__LocalPath__sampled_pt_x,  // get_const(index) function pointer
    get_function__LocalPath__sampled_pt_x,  // get(index) function pointer
    resize_function__LocalPath__sampled_pt_x  // resize(index) function pointer
  },
  {
    "sampled_pt_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, sampled_pt_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__sampled_pt_y,  // size() function pointer
    get_const_function__LocalPath__sampled_pt_y,  // get_const(index) function pointer
    get_function__LocalPath__sampled_pt_y,  // get(index) function pointer
    resize_function__LocalPath__sampled_pt_y  // resize(index) function pointer
  },
  {
    "sampled_pt_th",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, sampled_pt_th),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPath__sampled_pt_th,  // size() function pointer
    get_const_function__LocalPath__sampled_pt_th,  // get_const(index) function pointer
    get_function__LocalPath__sampled_pt_th,  // get(index) function pointer
    resize_function__LocalPath__sampled_pt_th  // resize(index) function pointer
  },
  {
    "nseg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, nseg),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "resolution",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::LocalPath, resolution),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LocalPath_message_members = {
  "mpcc_interfaces::msg",  // message namespace
  "LocalPath",  // message name
  9,  // number of fields
  sizeof(mpcc_interfaces::msg::LocalPath),
  LocalPath_message_member_array,  // message members
  LocalPath_init_function,  // function to initialize message memory (memory has to be allocated)
  LocalPath_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LocalPath_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LocalPath_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mpcc_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mpcc_interfaces::msg::LocalPath>()
{
  return &::mpcc_interfaces::msg::rosidl_typesupport_introspection_cpp::LocalPath_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mpcc_interfaces, msg, LocalPath)() {
  return &::mpcc_interfaces::msg::rosidl_typesupport_introspection_cpp::LocalPath_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
