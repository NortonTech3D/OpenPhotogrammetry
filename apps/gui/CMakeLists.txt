add_executable(op_app_gui
  src/main.cpp
)

target_link_libraries(op_app_gui
  PRIVATE
    OpenPhotogrammetry::camera
    OpenPhotogrammetry::meshing
)
