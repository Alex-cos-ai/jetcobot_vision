cmake_minimum_required(VERSION 3.22)
project(jetcobot_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(set_target_joints
  src/set_target_joints.cpp
)

ament_target_dependencies(set_target_joints
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(set_target_position
  src/set_target_position.cpp
)

ament_target_dependencies(set_target_position
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  set_target_joints
  set_target_position
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()