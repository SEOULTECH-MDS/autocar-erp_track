// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from adaptive_clustering:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "adaptive_clustering/msg/detail/cluster_array__rosidl_typesupport_introspection_c.h"
#include "adaptive_clustering/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "adaptive_clustering/msg/detail/cluster_array__functions.h"
#include "adaptive_clustering/msg/detail/cluster_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `clusters`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `clusters`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  adaptive_clustering__msg__ClusterArray__init(message_memory);
}

void adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_fini_function(void * message_memory)
{
  adaptive_clustering__msg__ClusterArray__fini(message_memory);
}

size_t adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__size_function__ClusterArray__clusters(
  const void * untyped_member)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return member->size;
}

const void * adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_const_function__ClusterArray__clusters(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__PointCloud2__Sequence * member =
    (const sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void * adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_function__ClusterArray__clusters(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  return &member->data[index];
}

void adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__fetch_function__ClusterArray__clusters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__PointCloud2 * item =
    ((const sensor_msgs__msg__PointCloud2 *)
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_const_function__ClusterArray__clusters(untyped_member, index));
  sensor_msgs__msg__PointCloud2 * value =
    (sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *value = *item;
}

void adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__assign_function__ClusterArray__clusters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__PointCloud2 * item =
    ((sensor_msgs__msg__PointCloud2 *)
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_function__ClusterArray__clusters(untyped_member, index));
  const sensor_msgs__msg__PointCloud2 * value =
    (const sensor_msgs__msg__PointCloud2 *)(untyped_value);
  *item = *value;
}

bool adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__resize_function__ClusterArray__clusters(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__PointCloud2__Sequence * member =
    (sensor_msgs__msg__PointCloud2__Sequence *)(untyped_member);
  sensor_msgs__msg__PointCloud2__Sequence__fini(member);
  return sensor_msgs__msg__PointCloud2__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(adaptive_clustering__msg__ClusterArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "clusters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(adaptive_clustering__msg__ClusterArray, clusters),  // bytes offset in struct
    NULL,  // default value
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__size_function__ClusterArray__clusters,  // size() function pointer
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_const_function__ClusterArray__clusters,  // get_const(index) function pointer
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__get_function__ClusterArray__clusters,  // get(index) function pointer
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__fetch_function__ClusterArray__clusters,  // fetch(index, &value) function pointer
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__assign_function__ClusterArray__clusters,  // assign(index, value) function pointer
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__resize_function__ClusterArray__clusters  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_members = {
  "adaptive_clustering__msg",  // message namespace
  "ClusterArray",  // message name
  2,  // number of fields
  sizeof(adaptive_clustering__msg__ClusterArray),
  adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_member_array,  // message members
  adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_init_function,  // function to initialize message memory (memory has to be allocated)
  adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_type_support_handle = {
  0,
  &adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_adaptive_clustering
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, adaptive_clustering, msg, ClusterArray)() {
  adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_type_support_handle.typesupport_identifier) {
    adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &adaptive_clustering__msg__ClusterArray__rosidl_typesupport_introspection_c__ClusterArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
