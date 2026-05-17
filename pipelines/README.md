# pipelines

Pipeline orchestration entrypoint for profile-specific photogrammetry workflows.

## Modules

- `reconstruction/`: shared staged gates, QA policy, observability, rollout
- `object-scan/`: close-range, high-detail object assumptions
- `aerial/`: geo-aware large-scene assumptions with tiling support

## Operating model

- All workflows run through strict fail-fast stage gates
- Profile behavior is configured via `configs/profiles/*.json`
- Sparse and dense QA outcomes determine whether downstream stages run
