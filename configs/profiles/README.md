# Profile Contracts (`configs/profiles`)

Profiles are strict runtime contracts consumed by reconstruction orchestration.

## Required fields

Every profile JSON must define:

- `name`
- `compute_backend_priority` (non-empty string array)
- `max_parallel_jobs` (`auto` or `max`)
- `image_cache_gb` (positive integer)
- `dense_reconstruction_quality` (`balanced`, `high`, `ultra`)
- `fail_fast_quality_gates` (boolean)
- `sparse_matching_strategy` (`adaptive`, `exhaustive`, `sequential`)
- `dense_confidence_filtering` (`disabled`, `enabled`, `aggressive`)
- `qa_report_level` (`standard`, `detailed`, `forensic`)
- `runtime_budget_ms` (positive integer stage runtime budget)

Optional:

- `unified_memory_mode` (boolean)

## Validation policy

- Profiles are rejected if required keys are missing.
- Profiles are rejected if semantic values are outside allowed sets.
- Valid profiles must remain backward-compatible with manifest/schema version `v1` contracts.

## Override hierarchy

Final runtime policy must resolve in this order:

1. built-in defaults
2. profile file
3. run-level overrides
4. stage-level overrides

Resolved values must be reflected in run manifests and QA outputs.

## Deprecation policy

- Profiles may add new keys only in a backward-compatible way.
- Existing keys/values may be deprecated only with migration guidance and a compatibility period.
- Removing a key/value requires a schema/version upgrade and migration documentation.
