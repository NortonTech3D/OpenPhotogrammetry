# Build Overview

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

## Notes
- Apple Silicon builds use `cmake/toolchains/apple-silicon.cmake`.
- Universal macOS binaries can use the `apple-universal-debug` preset.
