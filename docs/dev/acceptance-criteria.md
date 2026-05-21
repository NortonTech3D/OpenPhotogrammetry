# Production Acceptance Criteria

This document is the strict product specification for OpenPhotogrammetry production readiness.
A change is acceptable only when all mandatory criteria in this file are met.

## 1) Product output contract (measurable)

Every reconstruction run MUST emit a deterministic run manifest and QA report with the following measurable outputs:

- **Accuracy**
  - Mean reprojection error <= `1.0 px`
  - P95 reprojection error <= `2.5 px`
  - If control constraints are present: scale/georeference residual <= `1%` relative error (or <= `2x` GSD)
- **Completeness**
  - Sparse stage: registered view ratio >= `0.80`
  - Dense stage: valid dense coverage >= `0.85` over intended surface region
  - Texturing stage: UV/texture coverage >= `0.95` of mesh surface
- **Runtime budget**
  - Stage runtime must stay within configured profile budget
  - Total runtime must be reported in run manifest
- **Memory budget**
  - Peak per-stage memory must stay within profile memory policy
  - Violations are classified as `resource_exhaustion`
- **Failure behavior**
  - Failures MUST include: failure class, failed stage, attempts, and actionable diagnostic message
  - Partial progress MUST be checkpointed and resumable when possible

## 2) Non-functional requirements (mandatory)

- **Determinism / reproducibility**
  - Run ID generation is deterministic from project + dataset + profile.
  - Run manifests and QA reports include profile identity and stage metrics.
- **Crash recovery**
  - Stage checkpoint entries are persisted as artifacts.
  - Resume-from-stage execution must reuse prior completed stage checkpoints.
- **Observability**
  - Stage metrics include runtime, peak memory, QA metric, and artifact reference.
  - Diagnostics include stage-level pass/fail reasons.
- **Upgrade compatibility**
  - Manifest version must be explicit and validated.
  - Readers must reject unsupported versions with explicit validation failure.
- **Platform parity**
  - Linux, Windows, macOS, and Apple Silicon presets remain part of CI support gates.

## 3) Module done criteria

All modules below are "done" only when their behavior is test-backed and linked into an end-to-end executable path.

- `libs/core`
  - Runtime/version readiness API stable.
- `libs/io`
  - Canonical manifests exist for dataset metadata, camera models, calibration state, sparse model, dense model, mesh, texture, QA report.
  - Manifest version validation is enforced.
  - Profile contracts are validated (schema and semantic field values).
  - Deterministic run IDs and stage-graph orchestration APIs are implemented.
- `libs/camera`
  - Calibration validity checks are exposed and consumed by pipeline stages.
- `libs/features`
  - Feature budget/readiness checks are exposed and consumed by quality stages.
- `libs/sfm`
  - Sparse initialization/readiness checks are exposed and consumed by sparse stages.
- `libs/mvs`
  - Dense readiness checks are exposed and consumed by dense stages.
- `libs/meshing`
  - Mesh readiness checks are exposed and consumed by meshing stage.
- `libs/texturing`
  - Texture readiness checks are exposed and consumed by meshing/texturing stage.
- `libs/geo`
  - Geospatial readiness checks are exposed for geo-enabled flows.

## 4) Pipeline stage done criteria (`pipelines/reconstruction`)

The reconstruction pipeline MUST execute this explicit stage graph:

1. `intake`
2. `quality_screening`
3. `capture_adequacy`
4. `sparse`
5. `sparse_qa`
6. `dense`
7. `dense_qa`
8. `meshing_texturing`
9. `packaging_export`

Each stage is done only when all are true:

- Stage entry is recorded in checkpoints.
- Artifacts are identified with deterministic artifact URIs.
- Runtime, peak memory, and QA metric are captured.
- Retry policy is applied (minimum one attempt).
- Failure class is explicit when stage fails.

Pipeline done criteria:

- Run succeeds all stages OR fails with explicit terminal stage and failure classification.
- Stage execution manifest is written atomically.
- Resume-from-stage input is validated and honored.

## 5) Profile contract done criteria (`configs/profiles/*.json`)

A profile is valid only when required keys and values are present:

- Required keys: `name`, `compute_backend_priority`, `max_parallel_jobs`, `image_cache_gb`, `dense_reconstruction_quality`, `fail_fast_quality_gates`, `sparse_matching_strategy`, `dense_confidence_filtering`, `qa_report_level`, `runtime_budget_ms`
- Allowed values:
  - `max_parallel_jobs`: `auto` | `max`
  - `dense_reconstruction_quality`: `balanced` | `high` | `ultra`
  - `sparse_matching_strategy`: `adaptive` | `exhaustive` | `sequential`
  - `dense_confidence_filtering`: `disabled` | `enabled` | `aggressive`
  - `qa_report_level`: `standard` | `detailed` | `forensic`

Override hierarchy (authoritative):

`defaults -> profile -> run override -> stage override`

All resolved values must be auditable in run outputs.

## 6) Application surface done criteria

- `apps/cli`, `apps/gui`, `apps/worker` must build and self-check successfully.
- Exit codes are strict (`0` success, non-zero failure, `2` invalid invocation when applicable).
- App-level behavior must not bypass core stage/policy contracts.

## 7) Validation gates (must pass)

- Local baseline:
  - `cmake --preset linux-debug`
  - `cmake --build --preset linux-debug`
  - `ctest --preset linux-debug`
- CI platform gates:
  - `linux-debug`, `windows-debug`, `macos-debug`, `apple-silicon-debug`
- Linux quality gate:
  - `linux-asan`
