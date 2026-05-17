# OpenPhotogrammetry

Open source software for photogrammetry and other types of 3D scanning technology.

## Project foundation

This repository now includes a base, modular file system layout designed for scalable professional photogrammetry development.

### Top-level structure
- `apps/` — end-user applications (`cli`, `gui`, `worker`)
- `libs/` — reusable domain libraries (camera, features, SfM, MVS, meshing, texturing, geo, IO, core)
- `pipelines/` — orchestration for reconstruction workflows
- `tests/` — unit/integration/regression/performance scopes
- `configs/` — runtime profiles (`desktop`, `server`, `apple-silicon`)
- `cmake/` — toolchains and CMake support files
- `docs/` — architecture and developer documentation
- `packaging/` — distribution/installer definitions
- `tools/` — utility tooling
- `data/` — dataset metadata and references
- `third_party/` — vendored/pinned integrations

## Build system

- Root build: `CMakeLists.txt`
- Deterministic presets: `CMakePresets.json`
- Apple Silicon toolchain and profile support included

## Dependency management

- `vcpkg.json` is provided with a pinned baseline and core dependency set.
- See `docs/dev/dependencies.md` for details.

## Governance docs

- Repository layout: `docs/architecture/repository-layout.md`
- Build notes: `docs/dev/build.md`
- Dataset policy: `docs/dev/dataset-versioning.md`
