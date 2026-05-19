add_executable(op_app_worker
  src/main.cpp
)

target_link_libraries(op_app_worker
  PRIVATE
    OpenPhotogrammetry::io
    OpenPhotogrammetry::mvs
)
