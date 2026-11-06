cmake_minimum_required(VERSION 3.8)
project(armando_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)           # NUOVO
find_package(sensor_msgs REQUIRED)      # NUOVO
find_package(trajectory_msgs REQUIRED)  # NUOVO

# NUOVO: nodo di teleoperazione da tastiera
add_executable(armando_teleop src/armando_teleop.cpp)
ament_target_dependencies(armando_teleop rclcpp sensor_msgs trajectory_msgs)

install(TARGETS armando_teleop
  DESTINATION lib/${PROJECT_NAME}
)

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

install(
  DIRECTORY launch meshes urdf config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
