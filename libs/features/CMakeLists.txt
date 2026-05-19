add_library(op_features STATIC
  src/features.cpp
)

add_library(OpenPhotogrammetry::features ALIAS op_features)

target_include_directories(op_features
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_features
  PUBLIC
    OpenPhotogrammetry::camera
)
