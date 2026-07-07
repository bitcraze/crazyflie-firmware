# TDoA Measurement-Model Switch in the Bindings Replay

**Date:** 2026-07-07
**Branch:** tdoa-candidate-logging
**Status:** Approved

## Problem

The replay pipeline (`bindings/util/tdoa_replay.py` →
`bindings/util/estimator_kalman_emulator.py`) can only run recorded TDoA data
through the standard measurement model, `kalmanCoreUpdateWithTdoa`
(`src/modules/src/kalman_core/mm_tdoa.c`). The firmware also ships a robust
M-estimation-based update, `kalmanCoreRobustUpdateWithTdoa`
(`src/modules/src/kalman_core/mm_tdoa_robust.c`), selected on the drone with
the `kalman.robustTdoa` parameter — but it is not compiled into the Python
bindings and the emulator cannot call it. Comparing the two models on recorded
flights is a prerequisite for the outlier-filter work (that spec stays WIP;
this lands first, standalone).

## Facts that shape the design

- `kalmanCoreRobustUpdateWithTdoa(coreData, tdoa, outlierFilterState)` takes
  **no `now_ms`** and **never touches** `outlierFilterState` (it appears in the
  signature only). The robust path bypasses the integrator outlier filter
  entirely, both in firmware and in replay. No filter interaction to model.
- `mm_tdoa_robust.c` needs `arm_mat_inverse_f32` (via the `mat_inv` inline in
  `src/utils/interface/cf_math.h`), which is not among the CMSIS sources
  currently compiled by `bindings/setup.py`. Its other CMSIS dependencies
  (`mat_mult`, `mat_trans`) are already there.

## Design

### 1. Bindings exposure

- `bindings/setup.py`, `fw_sources`: add
  - `src/modules/src/kalman_core/mm_tdoa_robust.c`
  - `vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c`
- `bindings/cffirmware.i`: add `mm_tdoa_robust.h` to both the
  `%{ #include %}` block and the `%include` list (next to `mm_tdoa.h`).

### 2. Emulator model switch

`EstimatorKalmanEmulator(anchor_positions, tdoa_model='standard')`:

- `'standard'` → existing `kalmanCoreUpdateWithTdoa(coreData, tdoa, now_ms,
  outlierFilterState)` call, byte-for-byte unchanged.
- `'robust'` → `cffirmware.kalmanCoreRobustUpdateWithTdoa(coreData, tdoa,
  outlierFilterState)`.
- Any other value → `ValueError` in the constructor (fail fast, not at the
  first TDoA sample).

### 3. Replay seam

`tdoa_replay.replay(...)` gains a `params` key:

- `'tdoa_model'` (str, default `'standard'`): `'standard'` or `'robust'`,
  passed through to the emulator constructor.
- Resolved/validated before the `if not samples: return []` early-out so an
  unknown name raises even with empty input.
- Docstring `params` entry updated.

## Error handling

Single failure mode: unknown model name → `ValueError` naming the bad value
and the valid options, raised at emulator construction / before replay starts.

## Testing (TDD)

Extend the existing bindings smoke test (`test_python/`, runs via
`make test_python` against the real SWIG module):

1. **The switch switches:** identical synthetic IMU+TDoA input,
   `tdoa_model='robust'` vs `'standard'` → trajectories differ.
2. **No regression:** `params` without `'tdoa_model'` → trajectory identical
   to today's output (legacy path untouched).
3. **Fail fast:** `tdoa_model='huber'` → `ValueError`, even with empty
   `tdoa_samples`.

Existing suite stays green.

## Out of scope

- The pluggable outlier-filter refactor (separate WIP spec,
  `2026-07-07-tdoa-outlier-filters-design.md`) — this change does not touch
  `mm_tdoa.c` or the filter seam.
- Any firmware-side (.c) behavior change; this is bindings + Python only.
