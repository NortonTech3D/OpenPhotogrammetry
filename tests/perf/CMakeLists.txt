add_executable(op_perf_budget_smoke
  perf_budget_smoke.cpp
)

target_link_libraries(op_perf_budget_smoke
  PRIVATE
    OpenPhotogrammetry::features
)

add_test(
  NAME perf.feature_budget_smoke
  COMMAND op_perf_budget_smoke
)
