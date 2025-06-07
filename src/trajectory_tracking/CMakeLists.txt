cmake_minimum_required(VERSION 3.5)
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
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimedTrajectoryPoint.msg"
  "msg/TimedTrajectory.msg"
)

# Add include directory for headers
include_directories(
  include
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

# Add library
add_library(trajectory_tracking_lib
  src/trajectory_generator.cpp
  src/bspline.cpp
  src/path_smoother.cpp
  src/trajectory_parameterizer.cpp
)
ament_target_dependencies(trajectory_tracking_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_tracking_lib 
  yaml-cpp
  "${cpp_typesupport_target}"
)
target_include_directories(trajectory_tracking_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add pure pursuit controller executable
add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(pure_pursuit_controller 
  trajectory_tracking_lib
  "${cpp_typesupport_target}"
)

# Add trajectory generator executable
add_executable(trajectory_generator 
  src/trajectory_generator.cpp 
  src/bspline.cpp
  src/path_smoother.cpp
)
ament_target_dependencies(trajectory_generator
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  yaml-cpp
  visualization_msgs
)
target_link_libraries(trajectory_generator 
  trajectory_tracking_lib
  "${cpp_typesupport_target}"
)
target_include_directories(trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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
  trajectory_tracking_lib
  "${cpp_typesupport_target}"
)

# Install executables
install(TARGETS
  trajectory_generator
  pure_pursuit_controller
  trajectory_visualizer
  trajectory_tracking_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
