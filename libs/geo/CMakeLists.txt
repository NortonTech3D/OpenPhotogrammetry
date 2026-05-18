add_library(op_geo STATIC
  src/geo.cpp
)

add_library(OpenPhotogrammetry::geo ALIAS op_geo)

target_include_directories(op_geo
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(op_geo
  PUBLIC
    OpenPhotogrammetry::core
)
