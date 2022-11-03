// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_
#define MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_

#include "mpcc_interfaces/msg/detail/local_path__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mpcc_interfaces::msg::LocalPath>()
{
  return "mpcc_interfaces::msg::LocalPath";
}

template<>
inline const char * name<mpcc_interfaces::msg::LocalPath>()
{
  return "mpcc_interfaces/msg/LocalPath";
}

template<>
struct has_fixed_size<mpcc_interfaces::msg::LocalPath>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mpcc_interfaces::msg::LocalPath>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mpcc_interfaces::msg::LocalPath>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_
