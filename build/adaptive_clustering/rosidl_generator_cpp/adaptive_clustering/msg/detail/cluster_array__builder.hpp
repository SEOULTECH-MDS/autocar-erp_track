// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from adaptive_clustering:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#ifndef ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__BUILDER_HPP_
#define ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "adaptive_clustering/msg/detail/cluster_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace adaptive_clustering
{

namespace msg
{

namespace builder
{

class Init_ClusterArray_clusters
{
public:
  explicit Init_ClusterArray_clusters(::adaptive_clustering::msg::ClusterArray & msg)
  : msg_(msg)
  {}
  ::adaptive_clustering::msg::ClusterArray clusters(::adaptive_clustering::msg::ClusterArray::_clusters_type arg)
  {
    msg_.clusters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::adaptive_clustering::msg::ClusterArray msg_;
};

class Init_ClusterArray_header
{
public:
  Init_ClusterArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ClusterArray_clusters header(::adaptive_clustering::msg::ClusterArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ClusterArray_clusters(msg_);
  }

private:
  ::adaptive_clustering::msg::ClusterArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::adaptive_clustering::msg::ClusterArray>()
{
  return adaptive_clustering::msg::builder::Init_ClusterArray_header();
}

}  // namespace adaptive_clustering

#endif  // ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__BUILDER_HPP_
