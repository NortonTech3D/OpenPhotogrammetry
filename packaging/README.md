# packaging

Container baselines for stable compute runtime isolation.

## Docker images

- `packaging/docker/Dockerfile.rocm`: AMD ROCm-based runtime
- `packaging/docker/Dockerfile.intel`: Intel Compute Runtime + Level Zero

Both images install Python runtime dependencies used by the orchestration layer (`pyopf`, `requests`, `OpenImageIO`) and isolate pipeline execution from host display-driver churn.
