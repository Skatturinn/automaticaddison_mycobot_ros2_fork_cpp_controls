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

# Create executable
#add_executable(arm_gripper_loop_controller src/arm_gripper_loop_controller.cpp)
#ament_target_dependencies(arm_gripper_loop_controller rclcpp rclcpp_action control_msgs)

#add_executable(260309_in_out_demo src/260309_in_out_demo.cpp) # Takes the .cpp and makes a program
#ament_target_dependencies(260309_in_out_demo rclcpp rclcpp_action control_msgs) # aments the program to runtime with its dependencies so it can find header files

add_executable(secant src/secant.cpp) # Takes the .cpp and makes a program
ament_target_dependencies(secant rclcpp rclcpp_action control_msgs nlc_cpp_lib std_msgs sensor_msgs) # aments the program to runtime with its dependencies so it can find header files
target_link_libraries(secant Eigen3::Eigen)

add_executable(pid src/pid.cpp) #
ament_target_dependencies(pid rclcpp rclcpp_action control_msgs nlc_cpp_lib std_msgs sensor_msgs) #
target_link_libraries(pid Eigen3::Eigen)#

add_executable(ik_streamer_node src/ik_streamer_node.cpp)
ament_target_dependencies(ik_streamer_node 
  rclcpp 
  geometry_msgs 
  std_msgs 
  moveit_core 
  moveit_ros_planning_interface
)

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
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
