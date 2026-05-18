add_executable(op_app_cli
  src/main.cpp
)

target_link_libraries(op_app_cli
  PRIVATE
    OpenPhotogrammetry::sfm
    OpenPhotogrammetry::texturing
)
