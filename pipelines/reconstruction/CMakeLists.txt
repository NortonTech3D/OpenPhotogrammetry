add_executable(op_pipeline_reconstruction
  src/main.cpp
)

target_link_libraries(op_pipeline_reconstruction
  PRIVATE
    OpenPhotogrammetry::sfm
    OpenPhotogrammetry::texturing
)
