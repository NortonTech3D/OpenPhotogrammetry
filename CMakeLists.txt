cmake_minimum_required(VERSION 3.25)

project(OpenPhotogrammetry VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OP_ENABLE_TESTS "Enable test targets" ON)
option(OP_ENABLE_PIPELINES "Enable pipeline targets" ON)
option(OP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

if(MSVC)
  add_compile_options(/W4)
  if(OP_WARNINGS_AS_ERRORS)
    add_compile_options(/WX)
  endif()
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(OP_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
endif()

add_subdirectory(libs)
add_subdirectory(apps)

if(OP_ENABLE_PIPELINES)
  add_subdirectory(pipelines)
endif()

if(OP_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
