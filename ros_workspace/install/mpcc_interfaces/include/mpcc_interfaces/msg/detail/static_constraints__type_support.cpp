// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mpcc_interfaces/msg/detail/static_constraints__struct.hpp"
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

void StaticConstraints_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mpcc_interfaces::msg::StaticConstraints(_init);
}

void StaticConstraints_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mpcc_interfaces::msg::StaticConstraints *>(message_memory);
  typed_message->~StaticConstraints();
}

size_t size_function__StaticConstraints__ul_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__StaticConstraints__ul_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__StaticConstraints__ul_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__StaticConstraints__ul_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__StaticConstraints__ul_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__StaticConstraints__ul_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__StaticConstraints__ul_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__StaticConstraints__ul_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__StaticConstraints__lr_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__StaticConstraints__lr_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__StaticConstraints__lr_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__StaticConstraints__lr_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__StaticConstraints__lr_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__StaticConstraints__lr_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__StaticConstraints__lr_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__StaticConstraints__lr_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember StaticConstraints_message_member_array[5] = {
  {
    "point_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::StaticConstraints, point_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ul_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::StaticConstraints, ul_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__StaticConstraints__ul_x,  // size() function pointer
    get_const_function__StaticConstraints__ul_x,  // get_const(index) function pointer
    get_function__StaticConstraints__ul_x,  // get(index) function pointer
    resize_function__StaticConstraints__ul_x  // resize(index) function pointer
  },
  {
    "ul_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::StaticConstraints, ul_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__StaticConstraints__ul_y,  // size() function pointer
    get_const_function__StaticConstraints__ul_y,  // get_const(index) function pointer
    get_function__StaticConstraints__ul_y,  // get(index) function pointer
    resize_function__StaticConstraints__ul_y  // resize(index) function pointer
  },
  {
    "lr_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::StaticConstraints, lr_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__StaticConstraints__lr_x,  // size() function pointer
    get_const_function__StaticConstraints__lr_x,  // get_const(index) function pointer
    get_function__StaticConstraints__lr_x,  // get(index) function pointer
    resize_function__StaticConstraints__lr_x  // resize(index) function pointer
  },
  {
    "lr_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mpcc_interfaces::msg::StaticConstraints, lr_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__StaticConstraints__lr_y,  // size() function pointer
    get_const_function__StaticConstraints__lr_y,  // get_const(index) function pointer
    get_function__StaticConstraints__lr_y,  // get(index) function pointer
    resize_function__StaticConstraints__lr_y  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers StaticConstraints_message_members = {
  "mpcc_interfaces::msg",  // message namespace
  "StaticConstraints",  // message name
  5,  // number of fields
  sizeof(mpcc_interfaces::msg::StaticConstraints),
  StaticConstraints_message_member_array,  // message members
  StaticConstraints_init_function,  // function to initialize message memory (memory has to be allocated)
  StaticConstraints_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t StaticConstraints_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &StaticConstraints_message_members,
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
get_message_type_support_handle<mpcc_interfaces::msg::StaticConstraints>()
{
  return &::mpcc_interfaces::msg::rosidl_typesupport_introspection_cpp::StaticConstraints_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mpcc_interfaces, msg, StaticConstraints)() {
  return &::mpcc_interfaces::msg::rosidl_typesupport_introspection_cpp::StaticConstraints_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
