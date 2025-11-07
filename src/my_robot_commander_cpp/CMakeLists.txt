cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(robot_moveit src/robot_moveit.cpp)
ament_target_dependencies(robot_moveit 
  rclcpp 
  moveit_ros_planning_interface)

add_executable(robot_commander src/robot_commander.cpp)
ament_target_dependencies(robot_commander 
  rclcpp 
  moveit_ros_planning_interface 
  example_interfaces 
  my_robot_interfaces)



install(TARGETS
  robot_moveit robot_commander
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
