add_test(
  NAME integration.cli_self_check
  COMMAND $<TARGET_FILE:op_app_cli>
)

add_test(
  NAME integration.gui_self_check
  COMMAND $<TARGET_FILE:op_app_gui>
)

add_test(
  NAME integration.worker_self_check
  COMMAND $<TARGET_FILE:op_app_worker>
)

add_test(
  NAME integration.reconstruction_pipeline_self_check
  COMMAND $<TARGET_FILE:op_pipeline_reconstruction>
)

add_test(
  NAME integration.aerial_pipeline_self_check
  COMMAND $<TARGET_FILE:op_pipeline_aerial>
)

add_test(
  NAME integration.object_scan_pipeline_self_check
  COMMAND $<TARGET_FILE:op_pipeline_object_scan>
)
