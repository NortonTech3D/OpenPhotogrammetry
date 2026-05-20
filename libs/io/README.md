# libs/io

`libs/io` defines production contracts for manifests, profile policies, and stage orchestration persistence.

## Canonical manifests

Versioned manifest structs are defined for:

- dataset metadata
- camera models
- calibration state
- sparse model
- dense model
- mesh
- texture
- QA report

## Contracts and compatibility

- Supported manifest version is validated explicitly.
- Unsupported versions fail with validation diagnostics.
- Profile JSON contracts are validated for required keys and allowed values.

## Orchestration support

`libs/io` provides:

- deterministic run IDs
- explicit reconstruction stage graph definition
- checkpoint/retry execution primitives
- failure classification taxonomy
- atomic run-manifest persistence for postmortem/restart workflows
