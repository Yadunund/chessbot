cmake_minimum_required(VERSION 3.22)
project(rerun_ros_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rerun_sdk REQUIRED)

add_library(rerun_bridge_component SHARED
  src/image_conversion.cpp
  src/rerun_bridge.cpp
)
target_include_directories(rerun_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rerun_bridge_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  rerun_sdk
)
rclcpp_components_register_node(rerun_bridge_component
  PLUGIN "rerun_ros_bridge::RerunBridge"
  EXECUTABLE rerun_bridge
)

install(TARGETS rerun_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_conversion test/test_image_conversion.cpp src/image_conversion.cpp)
  target_include_directories(test_image_conversion PRIVATE include)
  target_link_libraries(test_image_conversion rerun_sdk)
endif()

ament_package()
