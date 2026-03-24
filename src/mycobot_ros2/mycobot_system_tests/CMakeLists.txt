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

# Create executable
add_executable(arm_gripper_loop_controller src/arm_gripper_loop_controller.cpp)
ament_target_dependencies(arm_gripper_loop_controller rclcpp rclcpp_action control_msgs)

add_executable(260309_in_out_demo src/260309_in_out_demo.cpp) # Takes the .cpp and makes a program
ament_target_dependencies(260309_in_out_demo rclcpp rclcpp_action control_msgs) # aments the program to runtime with its dependencies so it can find header files

install (
  DIRECTORY scripts src
  DESTINATION share/${PROJECT_NAME}
)

install ( 
  TARGETS
  arm_gripper_loop_controller
  260309_in_out_demo # installs the program
  DESTINATION lib/${PROJECT_NAME}
  )

install (
  PROGRAMS
  scripts/arm_gripper_loop_controller.py
  DESTINATION lib/${PROJECT_NAME}
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
