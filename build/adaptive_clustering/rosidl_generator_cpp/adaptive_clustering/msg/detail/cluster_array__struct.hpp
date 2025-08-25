// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from adaptive_clustering:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#ifndef ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_HPP_
#define ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'clusters'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__adaptive_clustering__msg__ClusterArray __attribute__((deprecated))
#else
# define DEPRECATED__adaptive_clustering__msg__ClusterArray __declspec(deprecated)
#endif

namespace adaptive_clustering
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ClusterArray_
{
  using Type = ClusterArray_<ContainerAllocator>;

  explicit ClusterArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ClusterArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _clusters_type =
    std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>>;
  _clusters_type clusters;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__clusters(
    const std::vector<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>> & _arg)
  {
    this->clusters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    adaptive_clustering::msg::ClusterArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const adaptive_clustering::msg::ClusterArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      adaptive_clustering::msg::ClusterArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      adaptive_clustering::msg::ClusterArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__adaptive_clustering__msg__ClusterArray
    std::shared_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__adaptive_clustering__msg__ClusterArray
    std::shared_ptr<adaptive_clustering::msg::ClusterArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClusterArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->clusters != other.clusters) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClusterArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClusterArray_

// alias to use template instance with default allocator
using ClusterArray =
  adaptive_clustering::msg::ClusterArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace adaptive_clustering

#endif  // ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_HPP_
