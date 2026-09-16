cmake_minimum_required(VERSION 3.22)
project(chessbot_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(chessbot_interfaces REQUIRED)

add_library(board_monitor_component SHARED
  src/board_detector.cpp
  src/board_monitor.cpp
)
target_include_directories(board_monitor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(board_monitor_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${chessbot_interfaces_TARGETS}
)
rclcpp_components_register_node(board_monitor_component
  PLUGIN "chessbot_perception::BoardMonitor"
  EXECUTABLE board_monitor
)

install(TARGETS board_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
