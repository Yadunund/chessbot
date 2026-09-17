cmake_minimum_required(VERSION 3.22)
project(chessbot_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Piece meshes for simulation, generated from the piece set description.
set(PIECE_MESH_DIR ${CMAKE_CURRENT_BINARY_DIR}/pieces/meshes)
set(PIECE_MESHES)
foreach(kind k q r b n p)
  list(APPEND PIECE_MESHES ${PIECE_MESH_DIR}/${kind}.obj)
endforeach()
add_custom_command(
  OUTPUT ${PIECE_MESHES}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/pieces/make_meshes.py
          ${CMAKE_CURRENT_SOURCE_DIR}/pieces/pieces.json ${PIECE_MESH_DIR}
  DEPENDS pieces/make_meshes.py pieces/pieces.json
  COMMENT "Generating chess piece meshes"
)
add_custom_target(piece_meshes ALL DEPENDS ${PIECE_MESHES})

install(DIRECTORY urdf config worlds DESTINATION share/${PROJECT_NAME})
install(FILES pieces/pieces.json DESTINATION share/${PROJECT_NAME}/pieces)
install(FILES ${PIECE_MESHES} DESTINATION share/${PROJECT_NAME}/pieces/meshes)

ament_package()
