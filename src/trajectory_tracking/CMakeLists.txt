cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimedTrajectoryPoint.msg"
  "msg/TimedTrajectory.msg"
  DEPENDENCIES geometry_msgs
)

# Add include directory for headers
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
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

# Add the BSpline library
add_library(bspline
  src/bspline.cpp
)

ament_target_dependencies(bspline
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  yaml-cpp
)

# Add the S-curve velocity profile library
add_library(s_curve_velocity_profile
  src/s_curve_velocity_profile.cpp
)

ament_target_dependencies(s_curve_velocity_profile
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# Add the trajectory generator executable
add_executable(trajectory_generator
  src/trajectory_generator.cpp
)

target_link_libraries(trajectory_generator
  s_curve_velocity_profile
  bspline
  yaml-cpp
)

ament_target_dependencies(trajectory_generator
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

rosidl_target_interfaces(trajectory_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add the pure pursuit controller executable
add_executable(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(pure_pursuit_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

rosidl_target_interfaces(pure_pursuit_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add trajectory visualizer executable
add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
ament_target_dependencies(trajectory_visualizer
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(trajectory_visualizer 
  "${cpp_typesupport_target}"
)

# Install targets
install(TARGETS
  bspline
  trajectory_generator
  pure_pursuit_controller
  trajectory_visualizer
  s_curve_velocity_profile
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
