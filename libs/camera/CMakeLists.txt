add_library(op_camera STATIC
  src/camera.cpp
)

add_library(OpenPhotogrammetry::camera ALIAS op_camera)

target_include_directories(op_camera
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_camera
  PUBLIC
    OpenPhotogrammetry::core
)
