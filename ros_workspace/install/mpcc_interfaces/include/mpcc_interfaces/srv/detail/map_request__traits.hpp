// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__TRAITS_HPP_
#define MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__TRAITS_HPP_

#include "mpcc_interfaces/srv/detail/map_request__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mpcc_interfaces::srv::MapRequest_Request>()
{
  return "mpcc_interfaces::srv::MapRequest_Request";
}

template<>
inline const char * name<mpcc_interfaces::srv::MapRequest_Request>()
{
  return "mpcc_interfaces/srv/MapRequest_Request";
}

template<>
struct has_fixed_size<mpcc_interfaces::srv::MapRequest_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mpcc_interfaces::srv::MapRequest_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mpcc_interfaces::srv::MapRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'costmap'
#include "nav2_msgs/msg/detail/costmap__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mpcc_interfaces::srv::MapRequest_Response>()
{
  return "mpcc_interfaces::srv::MapRequest_Response";
}

template<>
inline const char * name<mpcc_interfaces::srv::MapRequest_Response>()
{
  return "mpcc_interfaces/srv/MapRequest_Response";
}

template<>
struct has_fixed_size<mpcc_interfaces::srv::MapRequest_Response>
  : std::integral_constant<bool, has_fixed_size<nav2_msgs::msg::Costmap>::value> {};

template<>
struct has_bounded_size<mpcc_interfaces::srv::MapRequest_Response>
  : std::integral_constant<bool, has_bounded_size<nav2_msgs::msg::Costmap>::value> {};

template<>
struct is_message<mpcc_interfaces::srv::MapRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mpcc_interfaces::srv::MapRequest>()
{
  return "mpcc_interfaces::srv::MapRequest";
}

template<>
inline const char * name<mpcc_interfaces::srv::MapRequest>()
{
  return "mpcc_interfaces/srv/MapRequest";
}

template<>
struct has_fixed_size<mpcc_interfaces::srv::MapRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<mpcc_interfaces::srv::MapRequest_Request>::value &&
    has_fixed_size<mpcc_interfaces::srv::MapRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<mpcc_interfaces::srv::MapRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<mpcc_interfaces::srv::MapRequest_Request>::value &&
    has_bounded_size<mpcc_interfaces::srv::MapRequest_Response>::value
  >
{
};

template<>
struct is_service<mpcc_interfaces::srv::MapRequest>
  : std::true_type
{
};

template<>
struct is_service_request<mpcc_interfaces::srv::MapRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mpcc_interfaces::srv::MapRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__TRAITS_HPP_
