cmake_minimum_required(VERSION 3.8)
project(week_1_lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_executable(publisher src/week1Lab_pub.cpp)
#ament_target_dependencies(publisher rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
add_executable(subscriber src/week1Lab.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  #publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
