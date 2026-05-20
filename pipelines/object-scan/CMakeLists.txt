add_executable(op_pipeline_object_scan
  src/main.cpp
)

target_link_libraries(op_pipeline_object_scan
  PRIVATE
    OpenPhotogrammetry::camera
    OpenPhotogrammetry::features
    OpenPhotogrammetry::meshing
)
