cmake_minimum_required(VERSION 3.8)
project(sprint2)

# Add compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable
add_executable(map_processor_node src/sprint2.cpp)

# Link the required libraries
ament_target_dependencies(map_processor_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  OpenCV  # OpenCV dependency
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# Ensure OpenCV is linked
target_link_libraries(map_processor_node ${OpenCV_LIBRARIES})

# Install the executable
install(TARGETS map_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
