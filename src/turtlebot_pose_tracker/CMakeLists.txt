cmake_minimum_required(VERSION 3.8)
project(turtlebot_pose_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable
add_executable(turtlebot_pose_tracker src/turtlebot_pose_tracker.cpp)
target_include_directories(turtlebot_pose_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot_pose_tracker PUBLIC c_std_99 cxx_std_17)  # Require C++17

# Link against the required libraries
ament_target_dependencies(turtlebot_pose_tracker
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS turtlebot_pose_tracker
  DESTINATION lib/${PROJECT_NAME})

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()