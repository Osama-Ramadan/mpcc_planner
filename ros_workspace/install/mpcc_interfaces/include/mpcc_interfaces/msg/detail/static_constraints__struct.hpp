// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mpcc_interfaces:msg/StaticConstraints.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_HPP_
#define MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__mpcc_interfaces__msg__StaticConstraints __attribute__((deprecated))
#else
# define DEPRECATED__mpcc_interfaces__msg__StaticConstraints __declspec(deprecated)
#endif

namespace mpcc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StaticConstraints_
{
  using Type = StaticConstraints_<ContainerAllocator>;

  explicit StaticConstraints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->point_type = "";
    }
  }

  explicit StaticConstraints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->point_type = "";
    }
  }

  // field types and members
  using _point_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _point_type_type point_type;
  using _ul_x_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _ul_x_type ul_x;
  using _ul_y_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _ul_y_type ul_y;
  using _lr_x_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _lr_x_type lr_x;
  using _lr_y_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _lr_y_type lr_y;

  // setters for named parameter idiom
  Type & set__point_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->point_type = _arg;
    return *this;
  }
  Type & set__ul_x(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->ul_x = _arg;
    return *this;
  }
  Type & set__ul_y(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->ul_y = _arg;
    return *this;
  }
  Type & set__lr_x(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->lr_x = _arg;
    return *this;
  }
  Type & set__lr_y(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->lr_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> *;
  using ConstRawPtr =
    const mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mpcc_interfaces__msg__StaticConstraints
    std::shared_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mpcc_interfaces__msg__StaticConstraints
    std::shared_ptr<mpcc_interfaces::msg::StaticConstraints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StaticConstraints_ & other) const
  {
    if (this->point_type != other.point_type) {
      return false;
    }
    if (this->ul_x != other.ul_x) {
      return false;
    }
    if (this->ul_y != other.ul_y) {
      return false;
    }
    if (this->lr_x != other.lr_x) {
      return false;
    }
    if (this->lr_y != other.lr_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const StaticConstraints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StaticConstraints_

// alias to use template instance with default allocator
using StaticConstraints =
  mpcc_interfaces::msg::StaticConstraints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mpcc_interfaces

#endif  // MPCC_INTERFACES__MSG__DETAIL__STATIC_CONSTRAINTS__STRUCT_HPP_
