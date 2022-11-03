// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_
#define MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_

#include "mpcc_interfaces/msg/detail/local_path__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace mpcc_interfaces
{

namespace msg
{

namespace builder
{

class Init_LocalPath_resolution
{
public:
  explicit Init_LocalPath_resolution(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  ::mpcc_interfaces::msg::LocalPath resolution(::mpcc_interfaces::msg::LocalPath::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_nseg
{
public:
  explicit Init_LocalPath_nseg(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_resolution nseg(::mpcc_interfaces::msg::LocalPath::_nseg_type arg)
  {
    msg_.nseg = std::move(arg);
    return Init_LocalPath_resolution(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_sampled_pt_th
{
public:
  explicit Init_LocalPath_sampled_pt_th(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_nseg sampled_pt_th(::mpcc_interfaces::msg::LocalPath::_sampled_pt_th_type arg)
  {
    msg_.sampled_pt_th = std::move(arg);
    return Init_LocalPath_nseg(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_sampled_pt_y
{
public:
  explicit Init_LocalPath_sampled_pt_y(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_sampled_pt_th sampled_pt_y(::mpcc_interfaces::msg::LocalPath::_sampled_pt_y_type arg)
  {
    msg_.sampled_pt_y = std::move(arg);
    return Init_LocalPath_sampled_pt_th(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_sampled_pt_x
{
public:
  explicit Init_LocalPath_sampled_pt_x(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_sampled_pt_y sampled_pt_x(::mpcc_interfaces::msg::LocalPath::_sampled_pt_x_type arg)
  {
    msg_.sampled_pt_x = std::move(arg);
    return Init_LocalPath_sampled_pt_y(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_qy
{
public:
  explicit Init_LocalPath_qy(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_sampled_pt_x qy(::mpcc_interfaces::msg::LocalPath::_qy_type arg)
  {
    msg_.qy = std::move(arg);
    return Init_LocalPath_sampled_pt_x(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_qx
{
public:
  explicit Init_LocalPath_qx(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_qy qx(::mpcc_interfaces::msg::LocalPath::_qx_type arg)
  {
    msg_.qx = std::move(arg);
    return Init_LocalPath_qy(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_waypoints_y
{
public:
  explicit Init_LocalPath_waypoints_y(::mpcc_interfaces::msg::LocalPath & msg)
  : msg_(msg)
  {}
  Init_LocalPath_qx waypoints_y(::mpcc_interfaces::msg::LocalPath::_waypoints_y_type arg)
  {
    msg_.waypoints_y = std::move(arg);
    return Init_LocalPath_qx(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

class Init_LocalPath_waypoints_x
{
public:
  Init_LocalPath_waypoints_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPath_waypoints_y waypoints_x(::mpcc_interfaces::msg::LocalPath::_waypoints_x_type arg)
  {
    msg_.waypoints_x = std::move(arg);
    return Init_LocalPath_waypoints_y(msg_);
  }

private:
  ::mpcc_interfaces::msg::LocalPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mpcc_interfaces::msg::LocalPath>()
{
  return mpcc_interfaces::msg::builder::Init_LocalPath_waypoints_x();
}

}  // namespace mpcc_interfaces

#endif  // MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_
