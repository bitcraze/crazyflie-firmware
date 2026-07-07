# TDoA Measurement-Model Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Python-bindings replay able to run recorded TDoA data through the firmware's robust M-estimation measurement model (`kalmanCoreRobustUpdateWithTdoa`) as an alternative to the standard `kalmanCoreUpdateWithTdoa`, selected per run via `params['tdoa_model']`.

**Architecture:** Three thin layers, one task each: (1) compile `mm_tdoa_robust.c` into the SWIG bindings, (2) a `tdoa_model` constructor switch in `EstimatorKalmanEmulator`, (3) a `params['tdoa_model']` key in `tdoa_replay.replay()` that plumbs through to the emulator. Spec: `docs/superpowers/specs/2026-07-07-tdoa-measurement-model-switch-design.md`.

**Tech Stack:** SWIG Python bindings (`make bindings_python`), pytest (`PYTHONPATH=build python3 -m pytest`), C firmware sources compiled unchanged.

## Global Constraints

- No firmware `.c`/`.h` behavior changes — bindings build config and Python only.
- The standard-model path stays byte-for-byte unchanged: same `kalmanCoreUpdateWithTdoa(coreData, tdoa, now_ms, outlierFilterState)` call as today.
- `kalmanCoreRobustUpdateWithTdoa(coreData, tdoa, outlierFilterState)` takes **no `now_ms`** and ignores `outlierFilterState` (signature-only) — call it exactly like this, do not "fix" the unused argument.
- Valid model names are exactly `'standard'` and `'robust'`; anything else raises `ValueError` before any sample is processed, even with empty input.
- All test commands run from the repo root (`/home/rik/dev/crazyflie-firmware`) with `PYTHONPATH=build`.
- Working branch: `tdoa-candidate-logging`.

---

### Task 1: Expose `kalmanCoreRobustUpdateWithTdoa` in the SWIG bindings

**Files:**
- Modify: `bindings/setup.py` (the `fw_sources` list, around line 45)
- Modify: `bindings/cffirmware.i` (the `%{ #include %}` block around line 27 and the `%include` list around line 48)
- Test: `test_python/test_kalman_core.py`

**Interfaces:**
- Consumes: existing binding symbols `kalmanCoreData_t`, `kalmanCoreParams_t`, `kalmanCoreDefaultParams`, `kalmanCoreInit(coreData, params, now_ms)`, `tdoaMeasurement_t`, `vec3_s`, `OutlierFilterTdoaState_t`, `outlierFilterTdoaReset`.
- Produces: `cffirmware.kalmanCoreRobustUpdateWithTdoa(coreData: kalmanCoreData_t, tdoa: tdoaMeasurement_t, outlierFilterState: OutlierFilterTdoaState_t) -> None` — Tasks 2 and 3 rely on this exact name and 3-argument signature.

- [ ] **Step 1: Write the failing test**

Append to `test_python/test_kalman_core.py`:

```python
def test_robust_tdoa_update_is_exposed_and_runs():
    import cffirmware

    core = cffirmware.kalmanCoreData_t()
    params = cffirmware.kalmanCoreParams_t()
    cffirmware.kalmanCoreDefaultParams(params)
    cffirmware.kalmanCoreInit(core, params, 0)

    pos_a = cffirmware.vec3_s()
    pos_a.x, pos_a.y, pos_a.z = 0.0, 0.0, 0.0
    pos_b = cffirmware.vec3_s()
    pos_b.x, pos_b.y, pos_b.z = 4.0, 0.0, 0.0

    tdoa = cffirmware.tdoaMeasurement_t()
    tdoa.anchorIdA = 0
    tdoa.anchorIdB = 1
    tdoa.anchorPositionA = pos_a
    tdoa.anchorPositionB = pos_b
    tdoa.distanceDiff = 0.1
    tdoa.stdDev = 0.15

    outlier_filter_state = cffirmware.OutlierFilterTdoaState_t()
    cffirmware.outlierFilterTdoaReset(outlier_filter_state)

    # The robust update takes no now_ms (see mm_tdoa_robust.h)
    cffirmware.kalmanCoreRobustUpdateWithTdoa(core, tdoa, outlier_filter_state)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_core.py::test_robust_tdoa_update_is_exposed_and_runs -v`
Expected: FAIL with `AttributeError: module 'cffirmware' has no attribute 'kalmanCoreRobustUpdateWithTdoa'`

- [ ] **Step 3: Add the sources to the bindings build**

In `bindings/setup.py`, in the `fw_sources` list, add one CMSIS line after the existing `arm_mat_trans_f32.c` entry and one module line after the existing `mm_tdoa.c` entry:

```python
    'vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c',
    'vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c',
```

```python
    "src/modules/src/kalman_core/mm_tdoa.c",
    "src/modules/src/kalman_core/mm_tdoa_robust.c",
```

(`arm_mat_inverse_f32` is needed by the `mat_inv` inline in `src/utils/interface/cf_math.h`, which `mm_tdoa_robust.c` uses; `mat_mult`/`mat_trans` are already in the list.)

In `bindings/cffirmware.i`, add `mm_tdoa_robust.h` in both places, right after `mm_tdoa.h`:

```c
#include "mm_tdoa.h"
#include "mm_tdoa_robust.h"
```

```c
%include "mm_tdoa.h"
%include "mm_tdoa_robust.h"
```

- [ ] **Step 4: Rebuild the bindings**

Run: `make bindings_python`
Expected: swig + compile finish without errors; `build/cffirmware.py` regenerated. (If it fails with a missing-symbol link error mentioning `arm_mat_inverse_f32`, the CMSIS line from Step 3 is missing.)

- [ ] **Step 5: Run test to verify it passes**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_core.py -v`
Expected: PASS (all tests in the file, including the two pre-existing fixture tests)

- [ ] **Step 6: Commit**

```bash
git add bindings/setup.py bindings/cffirmware.i test_python/test_kalman_core.py
git commit -m "feat: expose robust TDoA measurement model in Python bindings"
```

---

### Task 2: `tdoa_model` switch in `EstimatorKalmanEmulator`

**Files:**
- Modify: `bindings/util/estimator_kalman_emulator.py:17-31` (constructor) and `:112-124` (`_update_with_sample`)
- Test: `test_python/test_kalman_core.py`

**Interfaces:**
- Consumes: `cffirmware.kalmanCoreRobustUpdateWithTdoa(coreData, tdoa, outlierFilterState)` from Task 1.
- Produces: `EstimatorKalmanEmulator(anchor_positions, tdoa_model='standard')` — constructor raises `ValueError` for names outside `TDOA_MODELS = ('standard', 'robust')`. Module-level constant `TDOA_MODELS` in `bindings/util/estimator_kalman_emulator.py`. Task 3 relies on the kwarg name `tdoa_model` and on the constructor doing the validation.

- [ ] **Step 1: Write the failing tests**

Append to `test_python/test_kalman_core.py` (add `import math` and `import pytest` to the imports at the top of the file):

```python
def test_emulator_rejects_unknown_tdoa_model():
    with pytest.raises(ValueError, match='huber'):
        EstimatorKalmanEmulator(anchor_positions={}, tdoa_model='huber')


def test_emulator_robust_model_processes_a_tdoa_sample():
    import cffirmware

    pos_a = cffirmware.vec3_s()
    pos_a.x, pos_a.y, pos_a.z = 0.0, 0.0, 0.0
    pos_b = cffirmware.vec3_s()
    pos_b.x, pos_b.y, pos_b.z = 4.0, 0.0, 0.0
    anchor_positions = {0: pos_a, 1: pos_b}

    emulator = EstimatorKalmanEmulator(anchor_positions, tdoa_model='robust')
    samples = [
        ('estAcceleration',
         {'timestamp': 1000.0, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}),
        ('estGyroscope',
         {'timestamp': 1000.0, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}),
        ('estTDOA',
         {'timestamp': 1001.0, 'idA': 0, 'idB': 1, 'distanceDiff': 0.1}),
    ]

    state = None
    while len(samples):
        _, state = emulator.run_one_1khz_iteration(samples)

    assert math.isfinite(state.position.x)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_core.py -v -k "tdoa_model or robust_model"`
Expected: `test_emulator_rejects_unknown_tdoa_model` FAILS with `TypeError: __init__() got an unexpected keyword argument 'tdoa_model'` (and the robust test fails the same way).

- [ ] **Step 3: Implement the switch**

In `bindings/util/estimator_kalman_emulator.py`, add a module-level constant after the imports:

```python
TDOA_MODELS = ('standard', 'robust')
```

Change the constructor signature and add validation as the first lines of `__init__`:

```python
    def __init__(self, anchor_positions, tdoa_model='standard') -> None:
        if tdoa_model not in TDOA_MODELS:
            raise ValueError(
                f"unknown tdoa_model '{tdoa_model}', expected one of {TDOA_MODELS}")
        self.tdoa_model = tdoa_model
        self.anchor_positions = anchor_positions
```

(rest of `__init__` unchanged.)

In `_update_with_sample`, replace the single `kalmanCoreUpdateWithTdoa` call (line 124) with:

```python
            if self.tdoa_model == 'robust':
                # No now_ms; ignores the outlier filter state, like the
                # firmware does with kalman.robustTdoa = 1
                cffirmware.kalmanCoreRobustUpdateWithTdoa(
                    self.coreData, tdoa, self.outlierFilterState)
            else:
                cffirmware.kalmanCoreUpdateWithTdoa(
                    self.coreData, tdoa, now_ms, self.outlierFilterState)
```

The `else` branch must be the exact call that is there today.

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_core.py -v`
Expected: PASS (all tests, including the pre-existing fixture tests — they exercise the default `'standard'` path and prove it unchanged)

- [ ] **Step 5: Commit**

```bash
git add bindings/util/estimator_kalman_emulator.py test_python/test_kalman_core.py
git commit -m "feat: tdoa_model switch (standard/robust) in EstimatorKalmanEmulator"
```

---

### Task 3: `params['tdoa_model']` in the replay seam

**Files:**
- Modify: `bindings/util/tdoa_replay.py:102-134` (the `replay` function)
- Test: `test_python/test_tdoa_replay_smoke.py`

**Interfaces:**
- Consumes: `EstimatorKalmanEmulator(anchor_positions, tdoa_model=...)` from Task 2 (constructor raises `ValueError` on unknown names).
- Produces: `replay(anchor_positions, imu_samples, tdoa_samples, params)` accepting `params['tdoa_model']` (str, default `'standard'`). This is the documented seam analysis scripts call.

- [ ] **Step 1: Write the failing tests**

Append to `test_python/test_tdoa_replay_smoke.py` (the file already imports `pytest` and defines `_anchor_positions`, `_synthetic_imu`, `_synthetic_tdoa`, `_dist`, `TRUE_POS`, `POSITION_TOLERANCE_M`):

```python
def test_robust_model_converges_and_differs_from_standard():
    standard = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                      {'tdoa_std': 0.15, 'tdoa_model': 'standard'})
    robust = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                    {'tdoa_std': 0.15, 'tdoa_model': 'robust'})

    _, robust_final = robust[-1]
    assert _dist(robust_final, TRUE_POS) < POSITION_TOLERANCE_M
    assert robust != standard


def test_default_model_is_standard():
    default = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                     {'tdoa_std': 0.15})
    explicit = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                      {'tdoa_std': 0.15, 'tdoa_model': 'standard'})
    assert default == explicit


def test_unknown_model_fails_fast_even_with_empty_input():
    with pytest.raises(ValueError, match='huber'):
        replay(_anchor_positions(), [], [], {'tdoa_model': 'huber'})
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_replay_smoke.py -v`
Expected: `test_robust_model_converges_and_differs_from_standard` FAILS on `assert robust != standard` (the param is ignored, both replays are identical); `test_unknown_model_fails_fast_even_with_empty_input` FAILS with `DID NOT RAISE` (empty input returns `[]` before any validation). `test_default_model_is_standard` passes vacuously; the pre-existing convergence test passes.

- [ ] **Step 3: Implement the plumb-through**

In `bindings/util/tdoa_replay.py`, update the `replay` docstring `params` entry to:

```python
        params: optional dict of tuning parameters. Supported:
            'tdoa_std' (float, default 0.15): TDoA measurement std dev [m].
            'tdoa_model' (str, default 'standard'): TDoA measurement model.
                'standard' is kalmanCoreUpdateWithTdoa; 'robust' is the
                M-estimation kalmanCoreRobustUpdateWithTdoa, which bypasses
                the outlier filter (firmware behavior with kalman.robustTdoa=1).
```

Replace the body after the `from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator` import so the emulator is constructed (and the model name validated) *before* the empty-input early-out:

```python
    params = params or {}
    emulator = EstimatorKalmanEmulator(
        anchor_positions, tdoa_model=params.get('tdoa_model', 'standard'))
    emulator.TDOA_ENGINE_MEASUREMENT_NOISE_STD = params.get('tdoa_std', DEFAULT_TDOA_STD)

    tdoa_samples, n_skipped = filter_known_anchors(tdoa_samples, anchor_positions)
    if n_skipped:
        print(f'WARNING: skipped {n_skipped} TDoA samples referencing anchors '
              f'not in the anchors file')
    samples = merge_samples(imu_samples, tdoa_samples)
    if not samples:
        return []

    trajectory = []
    while len(samples):
        now_ms, state = emulator.run_one_1khz_iteration(samples)
        trajectory.append((now_ms, (state.position.x, state.position.y, state.position.z)))
    return trajectory
```

(The only structural change is moving the two emulator lines above the filter/merge/early-out block and adding the `tdoa_model` kwarg; everything else is today's code.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_replay_smoke.py test_python/test_tdoa_replay.py -v`
Expected: PASS (all)

- [ ] **Step 5: Run the full Python test suite**

Run: `make test_python`
Expected: PASS — no regressions anywhere else.

- [ ] **Step 6: Commit**

```bash
git add bindings/util/tdoa_replay.py test_python/test_tdoa_replay_smoke.py
git commit -m "feat: params['tdoa_model'] selects the robust TDoA model in replay"
```
