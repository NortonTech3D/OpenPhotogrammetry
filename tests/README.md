# Test Runbook

## Test scopes

- `tests/unit/`: module behavior and link checks
- `tests/integration/`: app + pipeline interaction checks
- `tests/regression/`: fixed dataset contract checks
- `tests/perf/`: lightweight performance smoke checks

## Run all tests (Linux debug)

```bash
cmake --preset linux-debug
cmake --build --preset linux-debug
ctest --preset linux-debug
```

## Run only acceptance gates

```bash
ctest --preset linux-debug -R "^(unit|integration|regression)\\."
```

## Run sanitizer tests (Linux)

```bash
cmake --preset linux-asan
cmake --build --preset linux-asan
ctest --preset linux-asan
```
