cmake_minimum_required(VERSION 3.22)
project(chessbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config worlds DESTINATION share/${PROJECT_NAME})

ament_package()
