# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${patchworkpp_DIR}/../../../include;/opt/ros/humble/include/rclcpp;/opt/ros/humble/include/ament_index_cpp;/opt/ros/humble/include/libstatistics_collector;/opt/ros/humble/include/rcl;/opt/ros/humble/include/rcpputils;/opt/ros/humble/include/rcutils;/opt/ros/humble/include/builtin_interfaces;/opt/ros/humble/include/rosgraph_msgs;/opt/ros/humble/include/rosidl_typesupport_cpp;/opt/ros/humble/include/rosidl_typesupport_c;/opt/ros/humble/include/rosidl_runtime_cpp;/opt/ros/humble/include/rcl_yaml_param_parser;/opt/ros/humble/include/statistics_msgs;/opt/ros/humble/include/tracetools;/opt/ros/humble/include/pcl_ros;/opt/ros/humble/include/pcl_conversions;/opt/ros/humble/include/message_filters;/opt/ros/humble/include/pcl_msgs;/opt/ros/humble/include/sensor_msgs;/opt/ros/humble/include/std_msgs;/usr/include/pcl-1.12;/usr/include/eigen3;/usr/include;/usr/include/ni;/usr/include/openni2;/opt/ros/humble/include/geometry_msgs;/opt/ros/humble/include/tf2;/opt/ros/humble/include/tf2_geometry_msgs;/opt/ros/humble/include/tf2_ros;/opt/ros/humble/include/rosidl_runtime_c;/opt/ros/humble/include/rosidl_typesupport_interface;/opt/ros/humble/include/rosidl_typesupport_fastrtps_c;/opt/ros/humble/include/rosidl_typesupport_fastrtps_cpp;/opt/ros/humble/include/rmw;/opt/ros/humble/include/rosidl_typesupport_introspection_c;/opt/ros/humble/include/rosidl_typesupport_introspection_cpp;/opt/ros/humble/include/rclcpp_components;/opt/ros/humble/include/class_loader;/opt/ros/humble/include/composition_interfaces")

# append include directories to patchworkpp_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'patchworkpp' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND patchworkpp_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
