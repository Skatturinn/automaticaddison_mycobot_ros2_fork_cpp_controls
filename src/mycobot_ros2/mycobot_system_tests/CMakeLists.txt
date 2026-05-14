# Author: el3403
# Function: This file handles the build configuration for the 'mycobot_system_tests' ROS2 package.
# Input: Source files in /src and /scripts, package dependencies (rclcpp, moveit, etc.).
# Output: Executable binaries (secant, pid, ik_streamer_node) and installed package artifacts.

cmake_minimum_required(VERSION 3.8)
project(mycobot_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlc_cpp_lib REQUIRED) # Our lab specific library
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ---------------------------------------------------------------------------------------
# START OF CONTRIBUTIONS BY el3403
# ---------------------------------------------------------------------------------------

# Function: Define and configure the 'secant' executable.
# Pseudo-code: 
#   1. Locate src/secant.cpp.
#   2. Compile into an executable named 'secant'.
#   3. Link ROS2 dependencies (rclcpp, messages) and the lab-specific 'nlc_cpp_lib'.
#   4. Link Eigen3 library for linear algebra support.
# Input: src/secant.cpp, Eigen3, ROS2 system libraries.
# Output: secant binary in the build directory.
add_executable(secant src/secant.cpp) 
ament_target_dependencies(secant rclcpp rclcpp_action control_msgs nlc_cpp_lib std_msgs sensor_msgs) 
target_link_libraries(secant Eigen3::Eigen)

# Function: Define and configure the 'pid' executable.
# Pseudo-code:
#   1. Locate src/pid.cpp.
#   2. Compile into an executable named 'pid'.
#   3. Link required controller and sensor message dependencies.
#   4. Link Eigen3 for matrix/vector operations.
# Input: src/pid.cpp, ROS2 communication libraries.
# Output: pid binary in the build directory.
add_executable(pid src/pid.cpp) 
ament_target_dependencies(pid rclcpp rclcpp_action control_msgs nlc_cpp_lib std_msgs sensor_msgs) 
target_link_libraries(pid Eigen3::Eigen)

# Function: Define and configure the 'ik_streamer_node'.
# Pseudo-code:
#   1. Compile src/ik_streamer_node.cpp.
#   2. Link MoveIt planning interface and geometry message libraries.
# Input: src/ik_streamer_node.cpp, MoveIt libraries.
# Output: ik_streamer_node binary.
add_executable(ik_streamer_node src/ik_streamer_node.cpp)
ament_target_dependencies(ik_streamer_node 
  rclcpp 
  geometry_msgs 
  std_msgs 
  moveit_core 
  moveit_ros_planning_interface
)

# Function: Install targets to the workspace.
# Pseudo-code: Define the destination for compiled binaries and python scripts so they are discoverable by 'ros2 run'.
# Input: Compiled binaries (pid, ik_streamer_node, secant) and python scripts.
# Output: Installed files in the install/share and install/lib directories.
install(TARGETS pid ik_streamer_node secant
  DESTINATION lib/${PROJECT_NAME}
)

install (
  PROGRAMS
  scripts/arm_gripper_loop_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------------------
# END OF CONTRIBUTIONS BY el3403
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
