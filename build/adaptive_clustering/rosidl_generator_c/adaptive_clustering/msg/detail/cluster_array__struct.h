// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from adaptive_clustering:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#ifndef ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_
#define ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'clusters'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/ClusterArray in the package adaptive_clustering.
typedef struct adaptive_clustering__msg__ClusterArray
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__PointCloud2__Sequence clusters;
} adaptive_clustering__msg__ClusterArray;

// Struct for a sequence of adaptive_clustering__msg__ClusterArray.
typedef struct adaptive_clustering__msg__ClusterArray__Sequence
{
  adaptive_clustering__msg__ClusterArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} adaptive_clustering__msg__ClusterArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ADAPTIVE_CLUSTERING__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_
