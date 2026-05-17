cmake_minimum_required(VERSION 3.25)

project(OpenPhotogrammetry VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OP_ENABLE_TESTS "Enable test targets" ON)
option(OP_ENABLE_PIPELINES "Enable pipeline targets" ON)

add_subdirectory(libs)
add_subdirectory(apps)

if(OP_ENABLE_PIPELINES)
  add_subdirectory(pipelines)
endif()

if(OP_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
