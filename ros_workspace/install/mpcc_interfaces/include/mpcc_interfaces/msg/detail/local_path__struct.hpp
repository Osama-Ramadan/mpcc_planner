// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mpcc_interfaces:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_
#define MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__mpcc_interfaces__msg__LocalPath __attribute__((deprecated))
#else
# define DEPRECATED__mpcc_interfaces__msg__LocalPath __declspec(deprecated)
#endif

namespace mpcc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPath_
{
  using Type = LocalPath_<ContainerAllocator>;

  explicit LocalPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nseg = 0l;
      this->resolution = 0l;
    }
  }

  explicit LocalPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nseg = 0l;
      this->resolution = 0l;
    }
  }

  // field types and members
  using _waypoints_x_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _waypoints_x_type waypoints_x;
  using _waypoints_y_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _waypoints_y_type waypoints_y;
  using _qx_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _qx_type qx;
  using _qy_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _qy_type qy;
  using _sampled_pt_x_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _sampled_pt_x_type sampled_pt_x;
  using _sampled_pt_y_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _sampled_pt_y_type sampled_pt_y;
  using _sampled_pt_th_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _sampled_pt_th_type sampled_pt_th;
  using _nseg_type =
    int32_t;
  _nseg_type nseg;
  using _resolution_type =
    int32_t;
  _resolution_type resolution;

  // setters for named parameter idiom
  Type & set__waypoints_x(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->waypoints_x = _arg;
    return *this;
  }
  Type & set__waypoints_y(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->waypoints_y = _arg;
    return *this;
  }
  Type & set__qx(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->qx = _arg;
    return *this;
  }
  Type & set__qy(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->qy = _arg;
    return *this;
  }
  Type & set__sampled_pt_x(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->sampled_pt_x = _arg;
    return *this;
  }
  Type & set__sampled_pt_y(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->sampled_pt_y = _arg;
    return *this;
  }
  Type & set__sampled_pt_th(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->sampled_pt_th = _arg;
    return *this;
  }
  Type & set__nseg(
    const int32_t & _arg)
  {
    this->nseg = _arg;
    return *this;
  }
  Type & set__resolution(
    const int32_t & _arg)
  {
    this->resolution = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mpcc_interfaces::msg::LocalPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const mpcc_interfaces::msg::LocalPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mpcc_interfaces::msg::LocalPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mpcc_interfaces::msg::LocalPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mpcc_interfaces__msg__LocalPath
    std::shared_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mpcc_interfaces__msg__LocalPath
    std::shared_ptr<mpcc_interfaces::msg::LocalPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPath_ & other) const
  {
    if (this->waypoints_x != other.waypoints_x) {
      return false;
    }
    if (this->waypoints_y != other.waypoints_y) {
      return false;
    }
    if (this->qx != other.qx) {
      return false;
    }
    if (this->qy != other.qy) {
      return false;
    }
    if (this->sampled_pt_x != other.sampled_pt_x) {
      return false;
    }
    if (this->sampled_pt_y != other.sampled_pt_y) {
      return false;
    }
    if (this->sampled_pt_th != other.sampled_pt_th) {
      return false;
    }
    if (this->nseg != other.nseg) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPath_

// alias to use template instance with default allocator
using LocalPath =
  mpcc_interfaces::msg::LocalPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mpcc_interfaces

#endif  // MPCC_INTERFACES__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_
