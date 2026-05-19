add_executable(op_pipeline_aerial
  src/main.cpp
)

target_link_libraries(op_pipeline_aerial
  PRIVATE
    OpenPhotogrammetry::features
    OpenPhotogrammetry::geo
)
