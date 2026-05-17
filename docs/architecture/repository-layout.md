# Repository Layout

This repository is structured around clear module boundaries to support long-term growth.

- `apps/`: user-facing applications (`cli`, `gui`, `worker`)
- `libs/`: reusable core libraries for photogrammetry and scanning
- `pipelines/`: orchestration layers for reconstruction workflows
- `tests/`: unit, integration, regression, and performance test scopes
- `configs/profiles/`: runtime and performance profiles (including Apple Silicon)
- `cmake/`: toolchains and reusable CMake infrastructure
- `packaging/`: installers and distribution descriptors
- `tools/`: developer and data conversion utilities
- `data/`: dataset metadata and references (large binaries excluded)
- `third_party/`: vendored or pinned external source integrations
