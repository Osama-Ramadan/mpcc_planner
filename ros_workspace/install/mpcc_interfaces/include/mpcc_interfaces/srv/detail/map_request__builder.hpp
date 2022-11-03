// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__BUILDER_HPP_
#define MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__BUILDER_HPP_

#include "mpcc_interfaces/srv/detail/map_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace mpcc_interfaces
{

namespace srv
{

namespace builder
{

class Init_MapRequest_Request_y_max
{
public:
  explicit Init_MapRequest_Request_y_max(::mpcc_interfaces::srv::MapRequest_Request & msg)
  : msg_(msg)
  {}
  ::mpcc_interfaces::srv::MapRequest_Request y_max(::mpcc_interfaces::srv::MapRequest_Request::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mpcc_interfaces::srv::MapRequest_Request msg_;
};

class Init_MapRequest_Request_y_min
{
public:
  explicit Init_MapRequest_Request_y_min(::mpcc_interfaces::srv::MapRequest_Request & msg)
  : msg_(msg)
  {}
  Init_MapRequest_Request_y_max y_min(::mpcc_interfaces::srv::MapRequest_Request::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_MapRequest_Request_y_max(msg_);
  }

private:
  ::mpcc_interfaces::srv::MapRequest_Request msg_;
};

class Init_MapRequest_Request_x_max
{
public:
  explicit Init_MapRequest_Request_x_max(::mpcc_interfaces::srv::MapRequest_Request & msg)
  : msg_(msg)
  {}
  Init_MapRequest_Request_y_min x_max(::mpcc_interfaces::srv::MapRequest_Request::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_MapRequest_Request_y_min(msg_);
  }

private:
  ::mpcc_interfaces::srv::MapRequest_Request msg_;
};

class Init_MapRequest_Request_x_min
{
public:
  Init_MapRequest_Request_x_min()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapRequest_Request_x_max x_min(::mpcc_interfaces::srv::MapRequest_Request::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_MapRequest_Request_x_max(msg_);
  }

private:
  ::mpcc_interfaces::srv::MapRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mpcc_interfaces::srv::MapRequest_Request>()
{
  return mpcc_interfaces::srv::builder::Init_MapRequest_Request_x_min();
}

}  // namespace mpcc_interfaces


namespace mpcc_interfaces
{

namespace srv
{

namespace builder
{

class Init_MapRequest_Response_costmap
{
public:
  Init_MapRequest_Response_costmap()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mpcc_interfaces::srv::MapRequest_Response costmap(::mpcc_interfaces::srv::MapRequest_Response::_costmap_type arg)
  {
    msg_.costmap = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mpcc_interfaces::srv::MapRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mpcc_interfaces::srv::MapRequest_Response>()
{
  return mpcc_interfaces::srv::builder::Init_MapRequest_Response_costmap();
}

}  // namespace mpcc_interfaces

#endif  // MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__BUILDER_HPP_
