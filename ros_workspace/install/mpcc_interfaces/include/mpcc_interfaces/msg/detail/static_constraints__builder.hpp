// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__BUILDER_HPP_
#define MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__BUILDER_HPP_

#include "mpcc_interfaces/msg/detail/static_constraints__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace mpcc_interfaces
{

namespace msg
{

namespace builder
{

class Init_StaticConstraints_lr_y
{
public:
  explicit Init_StaticConstraints_lr_y(::mpcc_interfaces::msg::StaticConstraints & msg)
  : msg_(msg)
  {}
  ::mpcc_interfaces::msg::StaticConstraints lr_y(::mpcc_interfaces::msg::StaticConstraints::_lr_y_type arg)
  {
    msg_.lr_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mpcc_interfaces::msg::StaticConstraints msg_;
};

class Init_StaticConstraints_lr_x
{
public:
  explicit Init_StaticConstraints_lr_x(::mpcc_interfaces::msg::StaticConstraints & msg)
  : msg_(msg)
  {}
  Init_StaticConstraints_lr_y lr_x(::mpcc_interfaces::msg::StaticConstraints::_lr_x_type arg)
  {
    msg_.lr_x = std::move(arg);
    return Init_StaticConstraints_lr_y(msg_);
  }

private:
  ::mpcc_interfaces::msg::StaticConstraints msg_;
};

class Init_StaticConstraints_ul_y
{
public:
  explicit Init_StaticConstraints_ul_y(::mpcc_interfaces::msg::StaticConstraints & msg)
  : msg_(msg)
  {}
  Init_StaticConstraints_lr_x ul_y(::mpcc_interfaces::msg::StaticConstraints::_ul_y_type arg)
  {
    msg_.ul_y = std::move(arg);
    return Init_StaticConstraints_lr_x(msg_);
  }

private:
  ::mpcc_interfaces::msg::StaticConstraints msg_;
};

class Init_StaticConstraints_ul_x
{
public:
  explicit Init_StaticConstraints_ul_x(::mpcc_interfaces::msg::StaticConstraints & msg)
  : msg_(msg)
  {}
  Init_StaticConstraints_ul_y ul_x(::mpcc_interfaces::msg::StaticConstraints::_ul_x_type arg)
  {
    msg_.ul_x = std::move(arg);
    return Init_StaticConstraints_ul_y(msg_);
  }

private:
  ::mpcc_interfaces::msg::StaticConstraints msg_;
};

class Init_StaticConstraints_point_type
{
public:
  Init_StaticConstraints_point_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StaticConstraints_ul_x point_type(::mpcc_interfaces::msg::StaticConstraints::_point_type_type arg)
  {
    msg_.point_type = std::move(arg);
    return Init_StaticConstraints_ul_x(msg_);
  }

private:
  ::mpcc_interfaces::msg::StaticConstraints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mpcc_interfaces::msg::StaticConstraints>()
{
  return mpcc_interfaces::msg::builder::Init_StaticConstraints_point_type();
}

}  // namespace mpcc_interfaces

#endif  // MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__BUILDER_HPP_
