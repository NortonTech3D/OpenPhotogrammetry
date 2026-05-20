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

add_executable(op_unit_library_behavior
  library_behavior.cpp
)

target_link_libraries(op_unit_library_behavior
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

foreach(lib_name IN ITEMS core io camera features sfm mvs meshing texturing geo)
  add_test(
    NAME unit.${lib_name}_behavior
    COMMAND op_unit_library_behavior ${lib_name}
  )
endforeach()

add_executable(op_unit_io_contracts
  io_contracts.cpp
)

target_link_libraries(op_unit_io_contracts
  PRIVATE
    OpenPhotogrammetry::io
)

add_test(
  NAME unit.io_contracts
  COMMAND op_unit_io_contracts
)
