# Build Overview

## Quick start (recommended)

Run configure, build, and test in one command on your current host:

```bash
cmake --workflow --preset host-debug
```

## Local configure/build

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug
ctest --preset linux-debug
```

## Apple Silicon

```bash
cmake --preset apple-silicon-debug
cmake --build --preset apple-silicon-debug
ctest --preset apple-silicon-debug
```

## Sanitizer validation (Linux)

```bash
cmake --preset linux-asan
cmake --build --preset linux-asan
ctest --preset linux-asan
```

## Notes
- Apple Silicon builds use `cmake/toolchains/apple-silicon.cmake`.
- Universal macOS binaries can use the `apple-universal-debug` preset.
- Warnings are errors by default (`OP_WARNINGS_AS_ERRORS=ON`).
- CI runs acceptance gates for `unit.*`, `integration.*`, and `regression.*` tests.
