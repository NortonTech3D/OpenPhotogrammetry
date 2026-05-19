add_executable(op_regression_dataset_validation
  dataset_regression.cpp
)

target_link_libraries(op_regression_dataset_validation
  PRIVATE
    OpenPhotogrammetry::io
    OpenPhotogrammetry::texturing
)

add_test(
  NAME regression.fixed_dataset_contract
  COMMAND op_regression_dataset_validation ${CMAKE_SOURCE_DIR}/data/regression/fixed-mini-dataset.json
)
