cmake_minimum_required(VERSION 3.22)
project(chessbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BoardState.msg
  msg/GameState.msg
  msg/Thought.msg
  srv/GetBoardState.srv
  srv/Reason.srv
  action/Calibrate.action
  DEPENDENCIES std_msgs builtin_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
