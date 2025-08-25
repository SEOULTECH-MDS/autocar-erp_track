cmake_minimum_required(VERSION 3.8)
project(autocar_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python files
install(PROGRAMS
  autocar_utils/time_plot.py
  autocar_utils/autocar_util.py
  autocar_utils/cubic_spline_interpolator.py
  autocar_utils/euler_from_quaternion.py
  autocar_utils/normalise_angle.py
  autocar_utils/utils.py
  autocar_utils/yaw_to_quaternion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
