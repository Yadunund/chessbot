cmake_minimum_required(VERSION 3.22)
project(chessbot_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(chessbot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(zenohc REQUIRED)

# The square classifier has no ROS dependencies, so it is tested on its own.
add_library(square_classifier STATIC src/square_classifier.cpp)
target_include_directories(square_classifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(square_classifier PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(board_monitor_component SHARED
  src/board_monitor.cpp
  src/kv_client.cpp
)
target_link_libraries(board_monitor_component PUBLIC
  square_classifier
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${chessbot_interfaces_TARGETS}
  nlohmann_json::nlohmann_json
  zenohc::lib
)
rclcpp_components_register_node(board_monitor_component
  PLUGIN "chessbot_perception::BoardMonitor"
  EXECUTABLE board_monitor
)

# ROS message packages export their Python bindings as link dependencies, which reference
# Python symbols a C++ binary cannot resolve. Drop the libraries nothing here uses.
foreach(target board_monitor_component board_monitor)
  if(APPLE)
    target_link_options(${target} PRIVATE -Wl,-dead_strip_dylibs)
  else()
    target_link_options(${target} PRIVATE -Wl,--as-needed)
  endif()
endforeach()

install(TARGETS square_classifier board_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_square_classifier test/test_square_classifier.cpp)
  target_link_libraries(test_square_classifier square_classifier)
endif()

ament_package()
