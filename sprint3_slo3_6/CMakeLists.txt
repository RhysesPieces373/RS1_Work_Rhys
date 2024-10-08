cmake_minimum_required(VERSION 3.8)
project(sprint3_slo3_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Declare a C++ executable
add_executable(cyl_det src/cylDet.cpp)

# Specify libraries to link against
ament_target_dependencies(cyl_det rclcpp sensor_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# Install targets
install(TARGETS cyl_det
  DESTINATION lib/${PROJECT_NAME})

ament_package()
