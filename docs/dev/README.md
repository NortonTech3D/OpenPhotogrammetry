# Developer Runbook

This runbook defines the baseline workflow for validating functional changes and preserving cross-platform behavior.

## Validation baseline

Use CMake presets for reproducible local validation:

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug
ctest --preset linux-debug
```

## CI matrix expectations

The primary CI matrix must remain green for:

- `linux-debug` (ubuntu-latest)
- `windows-debug` (windows-latest)
- `macos-debug` (macos-13)
- `apple-silicon-debug` (macos-14)

## Quality gates

- Compiler warnings are treated as errors by default (`OP_WARNINGS_AS_ERRORS=ON`).
- Linux sanitizer preset (`linux-asan`) runs AddressSanitizer and UndefinedBehaviorSanitizer in CI.
- CI uploads CTest logs as artifacts for failure analysis.

## Where to find detailed guidance

- Build details: `docs/dev/build.md`
- Dependency policy: `docs/dev/dependencies.md`
- Dataset policy: `docs/dev/dataset-versioning.md`
- Acceptance criteria: `docs/dev/acceptance-criteria.md`
- Test scopes and commands: `tests/README.md`
