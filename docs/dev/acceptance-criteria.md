# Acceptance Criteria

These checks define minimum functionality required before merge.

## Modules (`libs/*`)

Each module must expose at least one behavior-backed API that is validated by unit tests:

- `core`: runtime/version readiness
- `io`: manifest readability guard
- `camera`: valid calibration defaults
- `features`: viable feature budget
- `sfm`: reconstruction initialization readiness
- `mvs`: dense reconstruction readiness
- `meshing`: mesh generation readiness
- `texturing`: texturing readiness
- `geo`: WGS84 support

## Applications (`apps/*`)

Each app target must build as an executable and return success when its library self-checks pass:

- `op_app_cli`
- `op_app_gui`
- `op_app_worker`

Validated via integration tests:

- `integration.cli_self_check`
- `integration.gui_self_check`
- `integration.worker_self_check`

## Pipelines (`pipelines/*`)

Each pipeline target must build as an executable and pass its self-check:

- `op_pipeline_reconstruction`
- `op_pipeline_aerial`
- `op_pipeline_object_scan`

Validated via integration tests:

- `integration.reconstruction_pipeline_self_check`
- `integration.aerial_pipeline_self_check`
- `integration.object_scan_pipeline_self_check`

## Regression contract

Regression tests must validate the fixed dataset contract:

- Dataset file: `data/regression/fixed-mini-dataset.json`
- Required contract: schema id, image count, coordinate system
- Test: `regression.fixed_dataset_contract`

## Platform support gates

Changes should pass CI presets:

- `linux-debug`
- `windows-debug`
- `macos-debug`
- `apple-silicon-debug`

And Linux quality gate:

- `linux-asan`
