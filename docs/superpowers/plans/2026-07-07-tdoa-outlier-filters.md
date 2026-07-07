# Pluggable TDoA Outlier Filters Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the TDoA outlier filter swappable in the offline replay the same way selection policies are — six filters (`integrator`, `none`, `sanity`, `fixed`, `mad_window`, `pair_integrator`) — plus a selectable TDoA update method (`standard` | `robust`, the `mm_tdoa_robust` M-estimator), all from the CLI.

**Architecture:** A behavior-preserving refactor of `mm_tdoa.c` exposes the innovation and an unconditional update through the SWIG bindings, so Python owns the accept/reject decision while all float32 math stays in C. A new `bindings/util/tdoa_outlier.py` mirrors the `tdoa_selection.py` policy pattern (base class + factories + `make_outlier_filter`). The emulator gets an optional `outlier_filter` and a `tdoa_update` method; defaults keep today's C path byte-for-byte. `kalmanCoreRobustUpdateWithTdoa` is already in the bindings and ignores its `outlierFilterState` argument (on-drone `robustTdoa=1` runs ungated), so the robust method needs no C changes and composes with any filter.

**Tech Stack:** C (firmware, `src/modules/src/kalman_core/mm_tdoa.c`), SWIG bindings (`make bindings_python`), Python 3 + pytest (`make test_python` = `PYTHONPATH=build python3 -m pytest test_python`).

**Spec:** `docs/superpowers/specs/2026-07-07-tdoa-outlier-filters-design.md`

## Global Constraints

- The `mm_tdoa.c` refactor must be behavior-preserving: same float32 operations in the same order in `kalmanCoreUpdateWithTdoa`; both the integrator path and the `CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK` steps path preserved verbatim.
- Parity requirement: replaying identical input with `outlier_filter=None` (legacy path) and `'integrator'` (new seam) must produce *exactly equal* trajectories (no tolerance).
- Pure-Python filters must not import `cffirmware` at module import time — only `IntegratorFilter` may, lazily (same convention as `tdoa_replay.py`).
- Unknown filter name raises `ValueError` listing available names, same style as `make_policy` in `bindings/util/tdoa_selection.py:167-173`.
- Degenerate geometry (d0 or d1 == 0) → sample skipped, filter state untouched (matches current firmware where the filter is not consulted in that case).
- CLI defaults must reproduce today's output: `plot_tdoa.py --outlier-filter integrator`, `replay_tdoa.py --outlier-filters integrator`, both `--tdoa-update standard`.
- Robust equivalence requirement: `tdoa_update='robust'` with filter `'none'` must produce a trajectory *exactly equal* to the direct no-filter robust path (the seam must be a transparent wrapper there). Invalid `tdoa_update` value → `ValueError` at emulator construction.
- Rebuild bindings after any C change: `make bindings_python` (from repo root). Run Python tests with `PYTHONPATH=build python3 -m pytest test_python/<file> -v`.
- Filter default tunings (from spec): `fixed` gate 2.5·stdDev; `mad_window` window 50, k 5, MAD floor 0.5·stdDev, accept-all warmup until 20 samples, rejected innovations also enter the window; `pair_integrator` same constants as C integrator (size 300, force-open 10%, resume 90%, trigger 2.0·std, accept 2.5·std, dt clamp size/10).

---

### Task 1: Firmware seam — refactor `mm_tdoa.c`, expose innovation + unfiltered update

**Files:**
- Modify: `src/modules/src/kalman_core/mm_tdoa.c`
- Modify: `src/modules/interface/kalman_core/mm_tdoa.h`
- Test: `test_python/test_mm_tdoa_seam.py` (create)

**Interfaces:**
- Consumes: existing `kalmanCoreData_t`, `tdoaMeasurement_t`, `kalmanCoreScalarUpdate` (all already in bindings via `%include "kalman_core.h"` / `"mm_tdoa.h"` in `bindings/cffirmware.i` — no `.i` change needed).
- Produces (used by Task 3):
  - `float kalmanCoreTdoaInnovation(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa)` — returns `error = distanceDiff - (dB - dA)`; NaN when degenerate; touches no state.
  - `void kalmanCoreUpdateWithTdoaUnfiltered(kalmanCoreData_t* this, tdoaMeasurement_t* tdoa)` — measurement model + `kalmanCoreScalarUpdate`, no outlier gate; no-op when degenerate.

- [ ] **Step 1: Write the failing test**

Create `test_python/test_mm_tdoa_seam.py`:

```python
"""Tests for the mm_tdoa bindings seam: innovation + unfiltered update.

Requires the SWIG bindings: make bindings_python, run with PYTHONPATH=build.
"""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')


def _point(x, y, z):
    # point_t is a typedef of struct vec3_s; vec3_s is the name the bindings
    # expose (same pattern as test_tdoa_replay_smoke.py / loco_utils.py).
    p = cffirmware.vec3_s()
    p.x, p.y, p.z = x, y, z
    return p


def _tdoa(pa, pb, dd, std=0.15):
    t = cffirmware.tdoaMeasurement_t()
    t.anchorIdA = 0
    t.anchorIdB = 1
    t.anchorPositionA = _point(*pa)
    t.anchorPositionB = _point(*pb)
    t.distanceDiff = dd
    t.stdDev = std
    return t


def _init_core():
    core = cffirmware.kalmanCoreData_t()
    params = cffirmware.kalmanCoreParams_t()
    cffirmware.kalmanCoreDefaultParams(params)
    cffirmware.kalmanCoreInit(core, params, 0)
    return core


def _position(core):
    state = cffirmware.state_t()
    acc = cffirmware.Axis3f()
    cffirmware.kalmanCoreExternalizeState(core, state, acc)
    return (state.position.x, state.position.y, state.position.z)


def test_innovation_is_measured_minus_predicted():
    # Tag at origin (default init), anchors symmetric at +-1 m on x:
    # dA == dB, so predicted == 0 and the innovation equals distanceDiff.
    core = _init_core()
    tdoa = _tdoa(pa=(1.0, 0.0, 0.0), pb=(-1.0, 0.0, 0.0), dd=0.25)
    error = cffirmware.kalmanCoreTdoaInnovation(core, tdoa)
    assert error == pytest.approx(0.25, abs=1e-6)


def test_innovation_is_nan_when_tag_is_at_an_anchor():
    # Anchor A at the origin == the initial tag position, so dA == 0
    # (degenerate). Firmware skips these samples; the seam reports NaN.
    core = _init_core()
    tdoa = _tdoa(pa=(0.0, 0.0, 0.0), pb=(2.0, 0.0, 0.0), dd=0.5)
    assert math.isnan(cffirmware.kalmanCoreTdoaInnovation(core, tdoa))


def test_unfiltered_update_moves_the_state():
    core = _init_core()
    before = _position(core)
    tdoa = _tdoa(pa=(1.0, 0.0, 0.0), pb=(-1.0, 0.0, 0.0), dd=0.25)
    cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(core, tdoa)
    assert _position(core) != before


def test_unfiltered_update_is_a_noop_when_degenerate():
    core = _init_core()
    before = _position(core)
    tdoa = _tdoa(pa=(0.0, 0.0, 0.0), pb=(2.0, 0.0, 0.0), dd=0.5)
    cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(core, tdoa)
    assert _position(core) == before
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_mm_tdoa_seam.py -v`
Expected: FAIL / ERROR with `AttributeError: module 'cffirmware' has no attribute 'kalmanCoreTdoaInnovation'` (all 4 tests). If it errors with "No module named cffirmware" the whole file is skipped — build bindings first: `make bindings_python`.

- [ ] **Step 3: Refactor `mm_tdoa.c`**

Replace the body of `src/modules/src/kalman_core/mm_tdoa.c` from `void kalmanCoreUpdateWithTdoa(...)` (line 33) to the end of that function (line 94) with:

```c
// Computes the innovation (error = measured - predicted) and the measurement
// jacobian h for a TDoA measurement, given the current state. Returns false
// in the degenerate case (the state coincides with an anchor position), in
// which case h is left zeroed and the measurement must be skipped.
static bool measurementModel(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa, float* error, float h[KC_STATE_DIM]) {
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = this->S[KC_STATE_X];
  float y = this->S[KC_STATE_Y];
  float z = this->S[KC_STATE_Z];

  float x1 = tdoa->anchorPositions[1].x, y1 = tdoa->anchorPositions[1].y, z1 = tdoa->anchorPositions[1].z;
  float x0 = tdoa->anchorPositions[0].x, y0 = tdoa->anchorPositions[0].y, z0 = tdoa->anchorPositions[0].z;

  float dx1 = x - x1;
  float dy1 = y - y1;
  float dz1 = z - z1;

  float dy0 = y - y0;
  float dx0 = x - x0;
  float dz0 = z - z0;

  float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
  float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

  float predicted = d1 - d0;
  *error = measurement - predicted;

  if ((d0 == 0.0f) || (d1 == 0.0f)) {
    return false;
  }

  h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
  h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
  h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);
  return true;
}

void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return;
  }
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

#if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
  vector_t jacobian = {
    .x = h[KC_STATE_X],
    .y = h[KC_STATE_Y],
    .z = h[KC_STATE_Z],
  };

  point_t estimatedPosition = {
    .x = this->S[KC_STATE_X],
    .y = this->S[KC_STATE_Y],
    .z = this->S[KC_STATE_Z],
  };

  bool sampleIsGood = outlierFilterTdoaValidateSteps(tdoa, error, &jacobian, &estimatedPosition);
#else
  bool sampleIsGood = outlierFilterTdoaValidateIntegrator(outlierFilterState, tdoa, error, nowMs);
#endif

  if (sampleIsGood) {
    kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
  }
}

float kalmanCoreTdoaInnovation(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return NAN;
  }
  return error;
}

void kalmanCoreUpdateWithTdoaUnfiltered(kalmanCoreData_t* this, tdoaMeasurement_t* tdoa)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return;
  }
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
}
```

Note the parity-critical details:
- The original code computes `error` *before* the `d0/d1` check and only fills `h` inside it; `measurementModel` keeps that exact order (`*error` assigned before the early return). The float ops are identical, only the branch is inverted (`== 0` early-out instead of `!= 0` wrap).
- In the original, the degenerate case fell through without touching the outlier-filter state; the early `return` preserves that.
- `NAN` requires `<math.h>`; verify it is already included at the top of `mm_tdoa.c` (it should be, for `sqrtf`/`powf` — if the include list has no `#include <math.h>`, add it).

Add to `src/modules/interface/kalman_core/mm_tdoa.h` after the existing `kalmanCoreUpdateWithTdoa` declaration (line 32):

```c
// Innovation (error = measured - predicted distance difference) of a TDoA
// measurement against the current state, without updating anything.
// Returns NAN in the degenerate case (state coincides with an anchor), where
// the firmware skips the sample entirely. Used by the Python bindings to
// replay logged data with an externalized (pluggable) outlier filter.
float kalmanCoreTdoaInnovation(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa);

// TDoA measurement update with NO outlier filter: the caller has already
// decided the sample is good. No-op in the degenerate case. Bindings-replay
// companion of kalmanCoreTdoaInnovation; not used by the flight code path.
void kalmanCoreUpdateWithTdoaUnfiltered(kalmanCoreData_t* this, tdoaMeasurement_t* tdoa);
```

- [ ] **Step 4: Rebuild bindings and run the test**

Run: `make bindings_python && PYTHONPATH=build python3 -m pytest test_python/test_mm_tdoa_seam.py -v`
Expected: 4 passed.

- [ ] **Step 5: Verify the firmware still builds and existing tests pass**

Run: `make cf2_defconfig && make -j$(nproc)`
Expected: build completes without errors or new warnings from `mm_tdoa.c`.

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_core.py test_python/test_tdoa_replay_smoke.py -v`
Expected: all pass (legacy path untouched).

- [ ] **Step 6: Commit**

```bash
git add src/modules/src/kalman_core/mm_tdoa.c src/modules/interface/kalman_core/mm_tdoa.h test_python/test_mm_tdoa_seam.py
git commit -m "refactor: expose TDoA innovation + unfiltered update for replay seam"
```

---

### Task 2: `bindings/util/tdoa_outlier.py` — filter roster + factories (bindings-free TDD)

**Files:**
- Create: `bindings/util/tdoa_outlier.py`
- Test: `test_python/test_tdoa_outlier.py` (create)

**Interfaces:**
- Consumes: nothing from other tasks at module-import time. `IntegratorFilter` lazily uses `cffirmware.OutlierFilterTdoaState_t`, `outlierFilterTdoaReset`, `outlierFilterTdoaValidateIntegrator` (already exposed today).
- Produces (used by Tasks 3–4):
  - `class OutlierFilter`: attributes `name: str`; methods `reset() -> None`, `validate(tdoa, error, now_ms) -> bool`. `tdoa` is any object with `anchorIdA`, `anchorIdB`, `anchorPositionA{.x,.y,.z}`, `anchorPositionB{.x,.y,.z}`, `distanceDiff`, `stdDev` (in practice a `cffirmware.tdoaMeasurement_t`).
  - `FILTER_FACTORIES: dict[str, type]` with keys `integrator`, `none`, `sanity`, `fixed`, `mad_window`, `pair_integrator`.
  - `make_outlier_filter(name) -> OutlierFilter` — `ValueError` on unknown name.

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_outlier.py`:

```python
"""Tests for the pluggable outlier filters (no cffirmware bindings required).

The 'integrator' filter wraps the real C filter and is exercised with the
bindings in test_tdoa_outlier_seam.py; here we only check its registration.
"""

import pytest

from bindings.util.tdoa_outlier import FILTER_FACTORIES, make_outlier_filter


class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Tdoa:
    """Stub with the tdoaMeasurement_t attributes the filters read."""

    def __init__(self, pa=(0.0, 0.0, 0.0), pb=(4.0, 0.0, 0.0), dd=0.0,
                 std=0.15, ids=(0, 1)):
        self.anchorPositionA = _Vec(*pa)
        self.anchorPositionB = _Vec(*pb)
        self.distanceDiff = dd
        self.stdDev = std
        self.anchorIdA, self.anchorIdB = ids


def test_all_spec_filters_are_registered():
    assert set(FILTER_FACTORIES) == {
        'integrator', 'none', 'sanity', 'fixed', 'mad_window', 'pair_integrator'}


def test_unknown_name_raises_value_error_listing_available():
    with pytest.raises(ValueError, match='mad_window'):
        make_outlier_filter('nope')


def test_none_accepts_anything():
    f = make_outlier_filter('none')
    assert f.validate(_Tdoa(dd=99.0), error=99.0, now_ms=0)


def test_sanity_rejects_physically_impossible_distance_diff():
    f = make_outlier_filter('sanity')
    # anchors 4 m apart: |dd| must be < 4
    assert f.validate(_Tdoa(dd=3.9), error=0.0, now_ms=0)
    assert not f.validate(_Tdoa(dd=4.1), error=0.0, now_ms=0)
    # the innovation is irrelevant to this filter
    assert f.validate(_Tdoa(dd=1.0), error=100.0, now_ms=0)


def test_fixed_gate_is_2p5_stddev():
    f = make_outlier_filter('fixed')
    t = _Tdoa(std=0.15)   # gate = 0.375
    assert f.validate(t, error=0.37, now_ms=0)
    assert not f.validate(t, error=0.38, now_ms=0)
    assert not f.validate(t, error=-0.38, now_ms=0)


def test_mad_window_accepts_everything_during_warmup():
    f = make_outlier_filter('mad_window')
    for i in range(20):
        assert f.validate(_Tdoa(), error=100.0 * i, now_ms=i)


def test_mad_window_rejects_far_from_median_after_warmup():
    f = make_outlier_filter('mad_window')
    for i in range(30):
        f.validate(_Tdoa(), error=0.0, now_ms=i)
    # MAD is floored at 0.5 * std = 0.075; gate = 5 * 0.075 = 0.375
    assert f.validate(_Tdoa(), error=0.3, now_ms=31)
    assert not f.validate(_Tdoa(), error=5.0, now_ms=32)


def test_mad_window_rejected_samples_still_enter_the_window():
    # A sustained level shift (real position jump) must eventually be accepted
    # because rejected innovations drag the median with them.
    f = make_outlier_filter('mad_window')
    for i in range(30):
        f.validate(_Tdoa(), error=0.0, now_ms=i)
    accepted = [f.validate(_Tdoa(), error=5.0, now_ms=100 + i) for i in range(60)]
    assert not accepted[0]
    assert accepted[-1]


def test_pair_integrator_isolates_anchor_pairs():
    f = make_outlier_filter('pair_integrator')
    # Converge pair (0, 1): good samples, dt capped at 30 ms each -> the
    # integrator reaches 300 (> resume level 270) after 10 samples and closes.
    for i in range(1, 11):
        assert f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=30 * i)
    # Pair (0, 1) is now closed: a large error is rejected...
    assert not f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=400)
    # ...but pair (0, 2) has fresh (open) state and lets it through.
    assert f.validate(_Tdoa(ids=(0, 2)), error=2.0, now_ms=400)


def test_pair_integrator_reset_reopens_all_pairs():
    f = make_outlier_filter('pair_integrator')
    for i in range(1, 11):
        f.validate(_Tdoa(ids=(0, 1)), error=0.0, now_ms=30 * i)
    assert not f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=400)
    f.reset()
    assert f.validate(_Tdoa(ids=(0, 1)), error=2.0, now_ms=500)


def test_factories_return_fresh_instances():
    a = make_outlier_filter('mad_window')
    b = make_outlier_filter('mad_window')
    assert a is not b
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test_python/test_tdoa_outlier.py -v` (from repo root; no PYTHONPATH needed — the module must be bindings-free)
Expected: collection ERROR with `ModuleNotFoundError: No module named 'bindings.util.tdoa_outlier'`.

- [ ] **Step 3: Implement the module**

Create `bindings/util/tdoa_outlier.py`:

```python
"""
Pluggable TDoA outlier filters for offline replay.

Companion of ``tdoa_selection.py`` (selection policies): where a policy decides
*which* candidate measurements to feed to the estimator, an outlier filter
decides — per measurement, given the innovation against the current Kalman
state — whether the update is applied at all. The live firmware hard-codes the
integrator filter inside ``kalmanCoreUpdateWithTdoa`` (mm_tdoa.c); the replay
seam externalizes that decision so alternative filter designs can be A/B tested
on identical logged data.

``validate(tdoa, error, now_ms)`` receives a ``tdoaMeasurement_t``-shaped
object (anchor ids/positions, distanceDiff, stdDev), the innovation ``error``
[m] computed by the firmware measurement model, and the sample time [ms].
Only ``IntegratorFilter`` needs the cffirmware bindings (imported lazily);
everything else is pure Python and unit-testable without them.
"""

from collections import deque


def _anchor_distance_sq(tdoa):
    a, b = tdoa.anchorPositionA, tdoa.anchorPositionB
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2


def _is_physically_possible(tdoa):
    # |distance difference| must be smaller than the anchor separation
    # (same check as isDistanceDiffSmallerThanDistanceBetweenAnchors in
    # outlierFilterTdoa.c).
    return tdoa.distanceDiff ** 2 < _anchor_distance_sq(tdoa)


class OutlierFilter:
    """Base class. ``validate`` returns True if the update should be applied."""
    name = 'base'

    def reset(self):
        """Reset internal state (called once before a replay run)."""

    def validate(self, tdoa, error, now_ms):
        raise NotImplementedError


class IntegratorFilter(OutlierFilter):
    """The real firmware filter (outlierFilterTdoaValidateIntegrator) via the
    bindings — the firmware-parity baseline. Requires ``tdoa`` to be an actual
    ``cffirmware.tdoaMeasurement_t``."""
    name = 'integrator'

    def __init__(self):
        import cffirmware  # lazy: only this filter needs the bindings
        self._cffirmware = cffirmware
        self._state = cffirmware.OutlierFilterTdoaState_t()
        self.reset()

    def reset(self):
        self._cffirmware.outlierFilterTdoaReset(self._state)

    def validate(self, tdoa, error, now_ms):
        return bool(self._cffirmware.outlierFilterTdoaValidateIntegrator(
            self._state, tdoa, error, int(now_ms)))


class NoneFilter(OutlierFilter):
    """Accept everything — the control group."""
    name = 'none'

    def validate(self, tdoa, error, now_ms):
        return True


class SanityFilter(OutlierFilter):
    """Only the physically-impossible check; no innovation gating."""
    name = 'sanity'

    def validate(self, tdoa, error, now_ms):
        return _is_physically_possible(tdoa)


class FixedGateFilter(OutlierFilter):
    """Static innovation gate at the integrator's closed-state acceptance
    level (2.5 sigma), with no adaptivity: a probe for whether the
    integrator's open/close machinery earns its keep."""
    name = 'fixed'
    GATE_STD_MULTIPLIER = 2.5

    def validate(self, tdoa, error, now_ms):
        return abs(error) < self.GATE_STD_MULTIPLIER * tdoa.stdDev


class MadWindowFilter(OutlierFilter):
    """Robust adaptive gate: reject innovations far from the running median,
    scaled by the median absolute deviation (MAD) of a sliding window.

    All innovations — accepted and rejected — enter the window, so a sustained
    level shift (a real position jump the estimator must follow) drags the
    median and reopens the gate; that is this filter's equivalent of the
    integrator's force-open recovery. The MAD is floored so a quiet window
    cannot collapse the gate, and everything is accepted until the window
    holds WARMUP samples.
    """
    name = 'mad_window'
    WINDOW = 50
    K = 5.0
    WARMUP = 20
    MAD_FLOOR_STD_MULTIPLIER = 0.5

    def __init__(self):
        self.reset()

    def reset(self):
        self._window = deque(maxlen=self.WINDOW)

    @staticmethod
    def _median(values):
        vals = sorted(values)
        n = len(vals)
        mid = n // 2
        if n % 2:
            return vals[mid]
        return 0.5 * (vals[mid - 1] + vals[mid])

    def validate(self, tdoa, error, now_ms):
        if len(self._window) < self.WARMUP:
            accepted = True
        else:
            med = self._median(self._window)
            mad = self._median(abs(e - med) for e in self._window)
            mad = max(mad, self.MAD_FLOOR_STD_MULTIPLIER * tdoa.stdDev)
            accepted = abs(error - med) <= self.K * mad
        self._window.append(error)
        return accepted


class PairIntegratorFilter(OutlierFilter):
    """Python port of the firmware integrator with one state per anchor pair,
    so a single bad anchor cannot force the global filter open. Same constants
    and update logic as outlierFilterTdoa.c, keyed by (idA, idB)."""
    name = 'pair_integrator'
    INTEGRATOR_SIZE = 300.0
    FORCE_OPEN_LEVEL = INTEGRATOR_SIZE * 0.1
    RESUME_ACTION_LEVEL = INTEGRATOR_SIZE * 0.9
    ACCEPT_STD_MULTIPLIER = 2.5
    TRIGGER_STD_MULTIPLIER = 2.0

    def __init__(self):
        self.reset()

    def reset(self):
        self._pairs = {}

    def validate(self, tdoa, error, now_ms):
        if not _is_physically_possible(tdoa):
            return False
        key = (int(tdoa.anchorIdA), int(tdoa.anchorIdB))
        s = self._pairs.setdefault(
            key, {'integrator': 0.0, 'is_open': True, 'latest_ms': 0})

        accepted_distance = tdoa.stdDev * self.ACCEPT_STD_MULTIPLIER
        trigger_distance = tdoa.stdDev * self.TRIGGER_STD_MULTIPLIER

        dt_ms = min(now_ms - s['latest_ms'], self.INTEGRATOR_SIZE / 10.0)
        if abs(error) < trigger_distance:
            s['integrator'] = min(s['integrator'] + dt_ms, self.INTEGRATOR_SIZE)
        else:
            s['integrator'] = max(s['integrator'] - dt_ms, 0.0)

        if s['is_open']:
            sample_is_good = True
            if s['integrator'] > self.RESUME_ACTION_LEVEL:
                s['is_open'] = False
        else:
            sample_is_good = abs(error) < accepted_distance
            if s['integrator'] < self.FORCE_OPEN_LEVEL:
                s['is_open'] = True

        s['latest_ms'] = now_ms
        return sample_is_good


# Factories (not instances) so each replay run gets fresh filter state.
FILTER_FACTORIES = {
    'integrator': IntegratorFilter,
    'none': NoneFilter,
    'sanity': SanityFilter,
    'fixed': FixedGateFilter,
    'mad_window': MadWindowFilter,
    'pair_integrator': PairIntegratorFilter,
}


def make_outlier_filter(name):
    try:
        return FILTER_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown outlier filter '{name}'. "
            f"Available: {', '.join(sorted(FILTER_FACTORIES))}")
```

Note: in the C filter the physically-impossible check *skips* the whole update block (integrator untouched, `sampleIsGood` false); `PairIntegratorFilter.validate` returns `False` before touching state to match.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test_python/test_tdoa_outlier.py -v`
Expected: 11 passed. (No bindings needed — this also proves the module is import-clean without cffirmware.)

- [ ] **Step 5: Commit**

```bash
git add bindings/util/tdoa_outlier.py test_python/test_tdoa_outlier.py
git commit -m "feat: pluggable TDoA outlier filters mirroring the selection-policy seam"
```

---

### Task 3: Emulator + replay wiring (filters + robust update), parity tests

**Files:**
- Modify: `bindings/util/estimator_kalman_emulator.py`
- Modify: `bindings/util/tdoa_replay.py`
- Test: `test_python/test_tdoa_outlier_seam.py` (create)

**Interfaces:**
- Consumes: `kalmanCoreTdoaInnovation` / `kalmanCoreUpdateWithTdoaUnfiltered` (Task 1), `make_outlier_filter` / `OutlierFilter.validate/reset` (Task 2), `cffirmware.kalmanCoreRobustUpdateWithTdoa` (already in the bindings; signature `(coreData, tdoa, outlierFilterState)` — the last argument is accepted but ignored by the C code).
- Produces (used by Task 4):
  - `EstimatorKalmanEmulator(anchor_positions, outlier_filter=None, tdoa_update='standard')` — defaults ⇒ legacy `kalmanCoreUpdateWithTdoa` path, unchanged; `tdoa_update` must be `'standard'` or `'robust'` (else `ValueError`).
  - `replay(anchor_positions, imu_samples, tdoa_samples, params)` accepts `params['outlier_filter']` (name string; absent ⇒ legacy path) and `params['tdoa_update']` (default `'standard'`).

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_outlier_seam.py`:

```python
"""Parity + integration tests for the outlier-filter replay seam.

THE key test: replaying identical input through the legacy C path
(outlier filter hard-wired in kalmanCoreUpdateWithTdoa) and through the new
seam with the 'integrator' filter must produce EXACTLY equal trajectories —
this is the drift alarm for the mm_tdoa.c refactor.

Requires the SWIG bindings: make bindings_python, run with PYTHONPATH=build.
"""

import math
import random

import pytest

cffirmware = pytest.importorskip('cffirmware')

from bindings.util.tdoa_replay import replay

# Same synthetic scene as test_tdoa_replay_smoke.py: 8 anchors on a
# 4 x 4 x 3 m box, stationary tag — but with outliers injected so the
# outlier filter actually rejects samples (otherwise parity is trivial).
ANCHOR_COORDS = {
    0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (0.0, 4.0, 0.0), 3: (4.0, 4.0, 0.0),
    4: (0.0, 0.0, 3.0), 5: (4.0, 0.0, 3.0), 6: (0.0, 4.0, 3.0), 7: (4.0, 4.0, 3.0),
}
TRUE_POS = (2.0, 2.0, 1.0)
DURATION_MS = 5000


def _anchor_positions():
    result = {}
    for anchor_id, (x, y, z) in ANCHOR_COORDS.items():
        p = cffirmware.vec3_s()
        p.x, p.y, p.z = x, y, z
        result[anchor_id] = p
    return result


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _synthetic_imu():
    samples = []
    for t in range(DURATION_MS):
        ts = float(t)
        samples.append(('estAcceleration',
                        {'timestamp': ts, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}))
        samples.append(('estGyroscope',
                        {'timestamp': ts, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}))
    return samples


def _synthetic_tdoa_with_outliers():
    # 100 Hz rotating pairs; every 10th sample corrupted by a large,
    # physically-possible error. Deterministic (seeded) so both replays see
    # byte-identical input.
    rng = random.Random(0)
    samples = []
    for k, t in enumerate(range(0, DURATION_MS, 10)):
        id_a = k % 8
        id_b = (k + 1) % 8
        dd = _dist(ANCHOR_COORDS[id_b], TRUE_POS) - _dist(ANCHOR_COORDS[id_a], TRUE_POS)
        dd += rng.gauss(0.0, 0.02)
        if k % 10 == 0:
            dd += rng.choice([-1.0, 1.0]) * 1.5
        samples.append(('estTDOA',
                        {'timestamp': float(t), 'idA': id_a, 'idB': id_b,
                         'distanceDiff': dd}))
    return samples


def test_integrator_via_seam_is_bit_identical_to_legacy_path():
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    legacy = replay(_anchor_positions(), list(imu), list(tdoa),
                    {'tdoa_std': 0.15})
    seam = replay(_anchor_positions(), list(imu), list(tdoa),
                  {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
    assert seam == legacy


def test_none_filter_gives_a_different_trajectory_with_outliers_present():
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    gated = replay(_anchor_positions(), list(imu), list(tdoa),
                   {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
    ungated = replay(_anchor_positions(), list(imu), list(tdoa),
                     {'tdoa_std': 0.15, 'outlier_filter': 'none'})
    assert ungated != gated


def test_every_registered_filter_replays_and_converges():
    from bindings.util.tdoa_outlier import FILTER_FACTORIES
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    for name in FILTER_FACTORIES:
        trajectory = replay(_anchor_positions(), list(imu), list(tdoa),
                            {'tdoa_std': 0.15, 'outlier_filter': name})
        assert len(trajectory) > 4000, name
        _, final_pos = trajectory[-1]
        # Outliers are injected, so allow a loose bound; 'none' is the worst.
        assert _dist(final_pos, TRUE_POS) < 1.5, name


def test_unknown_filter_name_fails_fast():
    with pytest.raises(ValueError, match='Unknown outlier filter'):
        replay(_anchor_positions(), _synthetic_imu()[:10], [],
               {'outlier_filter': 'nope'})


def test_unknown_update_method_fails_fast():
    with pytest.raises(ValueError, match='tdoa_update'):
        replay(_anchor_positions(), _synthetic_imu()[:10], [],
               {'tdoa_update': 'nope'})


def test_robust_with_none_filter_equals_direct_robust_path():
    # The innovation call is pure and 'none' accepts everything, so the seam
    # must be a transparent wrapper around the robust update.
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    direct = replay(_anchor_positions(), list(imu), list(tdoa),
                    {'tdoa_std': 0.15, 'tdoa_update': 'robust'})
    seam = replay(_anchor_positions(), list(imu), list(tdoa),
                  {'tdoa_std': 0.15, 'tdoa_update': 'robust',
                   'outlier_filter': 'none'})
    assert seam == direct


def test_robust_update_replays_and_converges():
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    trajectory = replay(_anchor_positions(), list(imu), list(tdoa),
                        {'tdoa_std': 0.15, 'tdoa_update': 'robust'})
    assert len(trajectory) > 4000
    _, final_pos = trajectory[-1]
    # The M-estimator downweights the injected outliers on its own.
    assert _dist(final_pos, TRUE_POS) < 1.0
```

Caveat for the robust tests: `mm_tdoa_robust.c` keeps state in `static` C locals (`x_err` and friends) that persist across calls within the process, and each call's first iteration reads the previous call's leftovers. Two otherwise identical replays therefore only match if the static state is identical when each starts. If the equivalence test fails with small differences, make it deterministic by *priming*: run one throwaway robust replay (same input) immediately before each of the two measured replays — identical throwaway runs leave identical static state behind. Add a comment in the test explaining the C static-state carryover. Try plain exact equality first; prime only if needed.

Note: `list(imu)` / `list(tdoa)` matter — `replay` consumes the sample list destructively (`samples.pop(0)` in the emulator), so each call gets its own copy.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_outlier_seam.py -v`
Expected: FAIL — `replay` currently ignores `params['outlier_filter']`, so `test_none_filter_gives_a_different_trajectory...` fails (both replays equal), and `test_unknown_filter_name_fails_fast` fails (no ValueError). The parity test passes vacuously at this point; that's expected.

- [ ] **Step 3: Wire the emulator**

In `bindings/util/estimator_kalman_emulator.py`:

1. Change the constructor signature, validate `tdoa_update`, and store both (line 17):

```python
    def __init__(self, anchor_positions, outlier_filter=None,
                 tdoa_update='standard') -> None:
        if tdoa_update not in ('standard', 'robust'):
            raise ValueError(
                f"tdoa_update must be 'standard' or 'robust', got '{tdoa_update}'")
        self.anchor_positions = anchor_positions
        self.outlier_filter = outlier_filter
        self.tdoa_update = tdoa_update
```
(keep the rest of `__init__` as is), and extend the class docstring with:

```
    An optional outlier_filter (see bindings/util/tdoa_outlier.py) externalizes
    the TDoA outlier decision: None keeps the firmware's built-in path
    (kalmanCoreUpdateWithTdoa, integrator filter in C), while a filter object
    routes each TDoA sample through kalmanCoreTdoaInnovation -> validate ->
    the chosen update. tdoa_update selects the measurement update:
    'standard' (mm_tdoa) or 'robust' (mm_tdoa_robust, the M-estimator; the
    firmware runs it ungated, so tdoa_update='robust' with outlier_filter=None
    matches the on-drone kalman.robustTdoa=1 path).
```

2. In `_lazy_init`, after `cffirmware.outlierFilterTdoaReset(self.outlierFilterState)` (line 92):

```python
        if self.outlier_filter is not None:
            self.outlier_filter.reset()
```

3. In `_update_with_sample`, replace the single line
`cffirmware.kalmanCoreUpdateWithTdoa(self.coreData, tdoa, now_ms, self.outlierFilterState)` (line 124) with:

```python
            if self.outlier_filter is None:
                if self.tdoa_update == 'robust':
                    # The robust update ignores the filter state (it is ungated
                    # in the firmware too: kalman.robustTdoa = 1).
                    cffirmware.kalmanCoreRobustUpdateWithTdoa(self.coreData, tdoa, self.outlierFilterState)
                else:
                    cffirmware.kalmanCoreUpdateWithTdoa(self.coreData, tdoa, now_ms, self.outlierFilterState)
            else:
                error = cffirmware.kalmanCoreTdoaInnovation(self.coreData, tdoa)
                # NaN = degenerate geometry; firmware skips the sample without
                # consulting the filter, so neither do we.
                if not math.isnan(error) and self.outlier_filter.validate(tdoa, error, now_ms):
                    if self.tdoa_update == 'robust':
                        cffirmware.kalmanCoreRobustUpdateWithTdoa(self.coreData, tdoa, self.outlierFilterState)
                    else:
                        cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(self.coreData, tdoa)
```

(`math` is already imported at the top of the file.)

- [ ] **Step 4: Wire `replay()`**

In `bindings/util/tdoa_replay.py`, `replay()` (line 102):

1. Resolve the filter name immediately after `params = params or {}` (line 118) — before any sample processing, so an unknown name fails fast:

```python
    params = params or {}
    outlier_filter = None
    filter_name = params.get('outlier_filter')
    if filter_name is not None:
        from bindings.util.tdoa_outlier import make_outlier_filter
        outlier_filter = make_outlier_filter(filter_name)
    tdoa_update = params.get('tdoa_update', 'standard')
```

2. Pass both to the emulator (line 127; the emulator constructor validates `tdoa_update` and raises `ValueError` on an unknown value — construct it *before* any sample processing so that failure is fast; with the innovation/merge code between resolution and construction, move the construction up next to the filter resolution if needed to satisfy `test_unknown_update_method_fails_fast`):

```python
    emulator = EstimatorKalmanEmulator(anchor_positions, outlier_filter=outlier_filter,
                                       tdoa_update=tdoa_update)
```

3. Update the `params` docstring entry to cover both keys:

```python
        params: optional dict of tuning parameters. Supported:
            'tdoa_std' (float, default 0.15): TDoA measurement std dev [m].
            'outlier_filter' (str, optional): outlier filter name (see
                bindings/util/tdoa_outlier.py). Absent -> the firmware's
                built-in C path (equivalent to 'integrator').
            'tdoa_update' (str, default 'standard'): TDoA measurement update:
                'standard' (mm_tdoa) or 'robust' (mm_tdoa_robust M-estimator;
                with no outlier_filter this matches kalman.robustTdoa = 1).
```

and the module docstring's example (line 10):

```python
    traj     = tdoa_replay.replay(anchor_positions, imu, tdoa,
                                  {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
```

- [ ] **Step 5: Run the tests**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_outlier_seam.py -v`
Expected: 7 passed. If the integrator parity test fails, the mm_tdoa.c refactor changed the float op order — diff `measurementModel` against the original math before debugging anything else. If only the robust equivalence test fails (small differences), see the static-state caveat under Step 1.

- [ ] **Step 6: Run the full Python suite**

Run: `make test_python`
Expected: everything passes (legacy-path users of `EstimatorKalmanEmulator` are untouched: `outlier_filter` defaults to `None`).

- [ ] **Step 7: Commit**

```bash
git add bindings/util/estimator_kalman_emulator.py bindings/util/tdoa_replay.py test_python/test_tdoa_outlier_seam.py
git commit -m "feat: route replay TDoA updates through pluggable outlier filters"
```

---

### Task 4: CLI — `--outlier-filters` grid in replay_tdoa.py, `--outlier-filter` in plot_tdoa.py

**Files:**
- Modify: `tools/usdlog/replay_tdoa.py`
- Modify: `tools/tdoa_plot/plot_tdoa.py`

**Interfaces:**
- Consumes: `replay(..., {'tdoa_std': ..., 'outlier_filter': <name>})` (Task 3).
- Produces: user-facing CLI; no downstream code consumers.

- [ ] **Step 1: Extend `replay_tdoa.py`**

1. `run_policy` (line 66) gains the filter name and update method:

```python
def run_policy(policy, anchor_positions, imu_samples, groups, tdoa_std,
               outlier_filter, tdoa_update):
    """Run the Kalman replay for one policy/outlier-filter combination."""
    tdoa_samples = apply_policy(policy, groups)
    return replay(anchor_positions, imu_samples, tdoa_samples,
                  {'tdoa_std': tdoa_std, 'outlier_filter': outlier_filter,
                   'tdoa_update': tdoa_update})
```

2. Add the arguments after `--policies` (line 142):

```python
    parser.add_argument('--outlier-filters', nargs='+', default=['integrator'],
                        help='Outlier filters to evaluate (each policy is '
                             'replayed once per filter; default: integrator, '
                             'the firmware filter)')
    parser.add_argument('--tdoa-update', choices=['standard', 'robust'],
                        default='standard',
                        help='TDoA measurement update for the whole run: '
                             'standard (mm_tdoa) or robust (mm_tdoa_robust '
                             'M-estimator; with --outlier-filters none this '
                             'matches the on-drone kalman.robustTdoa=1 path)')
```

Print the chosen method above the results table (just before the `header` line):

```python
    print(f'TDoA update method: {args.tdoa_update}')
```

3. Replace the results loop (lines 189–211) with a policies × filters grid; the row label becomes `policy/filter` and the column widens to fit it:

```python
    print()
    header = (f'{"policy/filter":<28} {"n":>7} {"rms[m]":>8} {"p95[m]":>8} '
              f'{"max[m]":>8} {"fly>thr":>8}')
    print(header)
    print('-' * len(header))
    for name in args.policies:
        for filter_name in args.outlier_filters:
            label = f'{name}/{filter_name}'
            policy = make_policy(name)
            trajectory = run_policy(policy, anchor_positions, imu_samples,
                                    groups, args.tdoa_std, filter_name,
                                    args.tdoa_update)
            metrics, _ = score_trajectory(trajectory, gt, args.flyaway_threshold)
            if (name == 'baseline' and filter_name == 'integrator'
                    and args.tdoa_update == 'standard' and live):
                live_metrics, _ = score_trajectory(trajectory, live, args.flyaway_threshold)
                if live_metrics['n']:
                    print(f'{"":28} baseline vs live stateEstimate: '
                          f'rms {live_metrics["rms"]:.3f} m, max {live_metrics["max"]:.3f} m '
                          f'(replay-vs-onboard divergence, spec F1)')
            if args.csv_dir:
                write_csv(os.path.join(args.csv_dir, f'replay_{name}_{filter_name}.csv'),
                          trajectory, gt)
            if metrics['n'] == 0:
                print(f'{label:<28} {"-":>7} {"(no overlap with ground truth)":>0}')
            else:
                print(f'{label:<28} {metrics["n"]:>7} {metrics["rms"]:>8.3f} '
                      f'{metrics["p95"]:>8.3f} {metrics["max"]:>8.3f} '
                      f'{metrics["flyaway_frac"]:>7.1%}'
                      f'   peak@{metrics["flyaway_t_ms"] / 1000.0:.1f}s')
```

4. Update the module docstring (lines 1–20): change the description line to
"...under several anchor-pair selection policies and TDoA outlier filters, ..."
and add a usage example:

```
    python3 -m tools.usdlog.replay_tdoa run01.bin --anchors anchors.yaml \
        --policies baseline all --outlier-filters integrator none mad_window
```

Note the CSV name change (`replay_<policy>_<filter>.csv`, was `replay_<policy>.csv`) — intentional, the filter is now part of the run identity.

- [ ] **Step 2: Extend `plot_tdoa.py`**

1. Add the argument after `--tdoa-std` (`tools/tdoa_plot/plot_tdoa.py:184`):

```python
    parser.add_argument('--outlier-filter', default='integrator',
                        help='TDoA outlier filter for the replay (see '
                             'bindings/util/tdoa_outlier.py; default: '
                             'integrator, the firmware filter)')
    parser.add_argument('--tdoa-update', choices=['standard', 'robust'],
                        default='standard',
                        help='TDoA measurement update: standard (mm_tdoa) or '
                             'robust (mm_tdoa_robust M-estimator)')
```

2. In `load_series` (line 79–84), pass them through and print them:

```python
    policy = make_policy(args.policy)
    tdoa_samples = apply_policy(policy, groups)
    print(f'Replaying {len(tdoa_samples)} TDoA + {len(imu_samples)} IMU samples '
          f'through the firmware Kalman core (policy: {args.policy}, '
          f'outlier filter: {args.outlier_filter}, '
          f'update: {args.tdoa_update}) ...')
    replayed = replay(anchor_positions, imu_samples, tdoa_samples,
                      {'tdoa_std': args.tdoa_std,
                       'outlier_filter': args.outlier_filter,
                       'tdoa_update': args.tdoa_update})
```

3. Include them in the title (line 197):

```python
    title = (f'{Path(args.logfile).name}  '
             f'(policy: {args.policy}, outlier filter: {args.outlier_filter}, '
             f'update: {args.tdoa_update})')
```

- [ ] **Step 3: Verify on the recorded hardware log**

Run (from repo root):

```bash
PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run_validation.bin \
    --anchors anchors.yaml \
    --policies baseline all \
    --outlier-filters integrator none sanity fixed mad_window pair_integrator
```

Expected: a 12-row table (`baseline/integrator`, `baseline/none`, …, `all/pair_integrator`) with plausible metrics; the `baseline/integrator` row must match the numbers of the pre-change tool (compare against the validation numbers in `tools/usdlog/README_tdoa_candidates.md`, "replay-vs-live rms 0.672 m / max 2.92 m" for the hand-moved run — the F1 line above the baseline row must be unchanged).

Also exercise the robust update on hardware data:

```bash
PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run_validation.bin \
    --anchors anchors.yaml --policies baseline \
    --outlier-filters none integrator --tdoa-update robust
```

Expected: 2 rows, plausible metrics, "TDoA update method: robust" printed above the table (the `baseline/none` row is the on-drone `robustTdoa=1` equivalent).

Also check the defaults reproduce today's output:

```bash
PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run_validation.bin --anchors anchors.yaml
```

Expected: 4 rows (`baseline/integrator` … `round_robin/integrator`) with the same metric values as before this change (only the label column differs).

And the plot tool (saves a PNG, no display needed):

```bash
cd tools/tdoa_plot && uv run plot_tdoa.py ../../run_validation.bin \
    --anchors ../../anchors.yaml --outlier-filter mad_window \
    --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/89a2dd08-ade1-4bfc-a1a1-28619ef95ad4/scratchpad/plot_mad.png && cd ../..
```

Expected: prints "… (policy: baseline, outlier filter: mad_window) …", saves the figure, exit 0.

- [ ] **Step 4: Run the Python suite once more**

Run: `make test_python`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add tools/usdlog/replay_tdoa.py tools/tdoa_plot/plot_tdoa.py
git commit -m "feat: outlier-filter selection in replay_tdoa and plot_tdoa CLIs"
```

---

### Task 5: Documentation + recorded validation run

**Files:**
- Modify: `tools/usdlog/README_tdoa_candidates.md`

**Interfaces:**
- Consumes: the working CLI from Task 4 (the validation numbers pasted here must come from an actual run).
- Produces: user documentation; no code consumers.

- [ ] **Step 1: Run the filter grid on the validation log and capture the output**

```bash
PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run_validation.bin \
    --anchors anchors.yaml --policies baseline \
    --outlier-filters integrator none sanity fixed mad_window pair_integrator \
    | tee /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/89a2dd08-ade1-4bfc-a1a1-28619ef95ad4/scratchpad/filter_validation.txt

PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run_validation.bin \
    --anchors anchors.yaml --policies baseline \
    --outlier-filters none integrator --tdoa-update robust \
    | tee /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/89a2dd08-ade1-4bfc-a1a1-28619ef95ad4/scratchpad/filter_validation_robust.txt
```

Keep both tables — they go into the README in the next step.

- [ ] **Step 2: Add the outlier-filter section to the README**

In `tools/usdlog/README_tdoa_candidates.md`, insert a new section directly after the "Selection policies" section (which ends with "Add your own in `bindings/util/tdoa_selection.py` (subclass `SelectionPolicy`).", around line 136):

```markdown
## Outlier filters

Orthogonal to the selection policy: the policy decides *which* candidates are
fed to the estimator; the outlier filter decides — per measurement, from the
innovation against the current Kalman state — whether the update is applied.
`replay_tdoa.py --outlier-filters` replays each policy once per filter;
`plot_tdoa.py --outlier-filter` picks one.

- `integrator` (default) — the real firmware filter
  (`outlierFilterTdoaValidateIntegrator`) driven through the replay seam;
  bit-identical to the built-in C path (guarded by a parity test,
  `test_python/test_tdoa_outlier_seam.py`).
- `none` — accept everything (control group).
- `sanity` — only the physically-impossible check
  (|distanceDiff| < anchor separation).
- `fixed` — static gate |innovation| < 2.5·std, no adaptivity.
- `mad_window` — reject innovations > 5·MAD from the running median of the
  last 50 (MAD floored at 0.5·std; accepts all until 20 samples; rejected
  samples still enter the window so a sustained level shift re-opens the gate).
- `pair_integrator` — the integrator logic with one state per anchor pair, so
  one bad anchor cannot force the global filter open.

Add your own in `bindings/util/tdoa_outlier.py` (subclass `OutlierFilter`).

Orthogonal to both: `--tdoa-update {standard,robust}` selects the measurement
update itself — `standard` (`mm_tdoa.c`) or `robust` (`mm_tdoa_robust.c`, the
UTIAS/DSL M-estimator). The firmware runs the robust update *ungated*
(`kalman.robustTdoa = 1`), so `--tdoa-update robust --outlier-filters none`
is the on-drone robust equivalent; combining it with a filter is an
offline-only experiment.

Validation run (`run_validation.bin`, baseline policy, all filters):

<paste the table from the tee'd run here — replace this line>

Robust update (`--tdoa-update robust`, baseline policy, filters none +
integrator):

<paste the robust table here — replace this line>
```

Both `<paste …>` lines MUST be replaced with the actual captured tables before committing.

- [ ] **Step 3: Update the stale Notes bullet**

In the "Notes / limitations" section of the same README, the first bullet currently reads:

> - The replay uses the real `kalman_core`, `outlierFilterTdoa` and `mm_tdoa` via
>   the `cffirmware` SWIG bindings (build them first; see `build_python.sh`).

Replace it with:

```markdown
- The replay uses the real `kalman_core` and `mm_tdoa` via the `cffirmware`
  SWIG bindings (build them first; see `build_python.sh`). The TDoA outlier
  filter is pluggable (`--outlier-filters`); the default `integrator` is the
  real firmware filter and is bit-identical to the built-in C path.
```

- [ ] **Step 4: Commit**

```bash
git add tools/usdlog/README_tdoa_candidates.md
git commit -m "docs: outlier-filter roster + hardware validation table"
```
