add_executable(op_unit_target_link_smoke
  target_link_smoke.cpp
)

target_link_libraries(op_unit_target_link_smoke
  PRIVATE
    OpenPhotogrammetry::core
    OpenPhotogrammetry::io
    OpenPhotogrammetry::camera
    OpenPhotogrammetry::features
    OpenPhotogrammetry::sfm
    OpenPhotogrammetry::mvs
    OpenPhotogrammetry::meshing
    OpenPhotogrammetry::texturing
    OpenPhotogrammetry::geo
)

add_test(
  NAME unit.target_link_smoke
  COMMAND op_unit_target_link_smoke
)
