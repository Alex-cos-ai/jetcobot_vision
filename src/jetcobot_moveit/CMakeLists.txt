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
install(
  PROGRAMS
    script/detect_marker_debug.py
    script/get_target_joint_angles.py
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS script/handeye_collect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME handeye_collect)
install(
  PROGRAMS script/handeye_solve.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME handeye_solve)
install(
  PROGRAMS script/handeye_solve.py script/handeye_collect.py
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS script/charuco_handeye_collect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME charuco_handeye_collect)
install(
  PROGRAMS script/charuco_handeye_solve.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME charuco_handeye_solve)
install(
  FILES script/charuco_board.py
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS script/pregrasp_move.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pregrasp_move)
install(
  PROGRAMS script/local_joint_camera_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME local_joint_camera_control)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(handeye_solve_test test/test_handeye_solve.py)
  ament_add_pytest_test(charuco_handeye_test test/test_charuco_handeye.py)
  ament_add_pytest_test(charuco_collector_test test/test_charuco_collector.py)
  ament_add_pytest_test(pregrasp_handeye_test test/test_pregrasp_handeye.py)
endif()

ament_package()
