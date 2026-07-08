# TDoA Diagnostics Data Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement phases 0–2 of `docs/superpowers/specs/2026-07-08-tdoa-analysis-views-design.md`: move experimental algorithms above the exposure/analysis cut line, add the two C/SWIG exposure helpers, instrument the replay seam with a `ReplayRecorder`, and add ground-truth math (`tdoa_truth.py`).

**Architecture:** `bindings/util/` + `src/` + `bindings/cffirmware.i` are the upstreamable exposure layer (no plots, no experimental algorithms); `tools/tdoa_plot/` is the only analysis directory. Three data products cross the line: candidate groups (exists), replay diagnostics (`ReplayRecorder`), truth labels (`tdoa_truth`). Phase 3 (views) is a separate plan: `2026-07-08-tdoa-analysis-views.md`.

**Tech Stack:** C (firmware helpers), SWIG (`bindings/cffirmware.i`), Python 3.10+, pytest (`test_python/`).

## Global Constraints

- Cut line: nothing under `bindings/`, `src/` may import matplotlib or contain experimental algorithms; experimental policies/filters live only in `tools/tdoa_plot/experiments.py`.
- Pure-Python modules in `bindings/util/` must import without the cffirmware bindings (lazy `import cffirmware` only inside classes/functions that need it).
- Firmware-parity classes state in their docstring that they mirror live firmware behavior; reference examples (`AllCandidatesPolicy`, `NoneFilter`) state they are kept as trivial reference examples of the plug-point contract.
- Tests: run `PYTHONPATH=build python3 -m pytest test_python/<file> -v` (same as `make test_python`). Bindings-dependent test files start with `cffirmware = pytest.importorskip('cffirmware')`.
- Rebuild bindings after C/SWIG changes: `make bindings_python` (from repo root).
- The full suite `make test_python` must pass at the end of every task.

---

### Task 1: `replay()` accepts outlier-filter instances

`params['outlier_filter']` currently only accepts a name resolved via `make_outlier_filter` inside `bindings/util`. Once experimental filters move above the line (Task 2), the seam cannot know their names — it must accept instances.

**Files:**
- Modify: `bindings/util/tdoa_replay.py:130-135` (filter resolution in `replay`)
- Test: `test_python/test_tdoa_outlier_seam.py`

**Interfaces:**
- Produces: `replay(anchor_positions, imu_samples, tdoa_samples, params)` where `params['outlier_filter']` is a core-registry name (`str`) **or** an `OutlierFilter` instance. Tasks 2, 8 and the views plan rely on instance-passing.

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_outlier_seam.py`:

```python
def test_filter_instance_in_params_matches_name():
    # The seam accepts an OutlierFilter instance, so filters that live outside
    # bindings/util (tools/tdoa_plot/experiments.py) can be replayed.
    from bindings.util.tdoa_outlier import IntegratorFilter
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    by_name = replay(_anchor_positions(), list(imu), list(tdoa),
                     {'tdoa_std': 0.15, 'outlier_filter': 'integrator'})
    by_instance = replay(_anchor_positions(), list(imu), list(tdoa),
                         {'tdoa_std': 0.15, 'outlier_filter': IntegratorFilter()})
    assert by_instance == by_name
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_outlier_seam.py::test_filter_instance_in_params_matches_name -v`
Expected: FAIL (`AttributeError` or `KeyError`/`TypeError` from `make_outlier_filter` receiving an instance)

- [ ] **Step 3: Implement**

In `bindings/util/tdoa_replay.py`, replace the filter-resolution block in `replay()`:

```python
    params = params or {}
    outlier_filter = params.get('outlier_filter')
    if isinstance(outlier_filter, str):
        from bindings.util.tdoa_outlier import make_outlier_filter
        outlier_filter = make_outlier_filter(outlier_filter)
```

Update the `'outlier_filter'` entry in the `replay` docstring:

```
            'outlier_filter' (str or OutlierFilter instance, optional): TDoA
                outlier filter. A string is resolved against the core registry
                in bindings/util/tdoa_outlier.py; an instance (e.g. an
                experimental filter from tools/tdoa_plot/experiments.py) is
                used as-is. Absent -> the firmware's built-in behavior
                (standard model: C integrator filter; robust model: ungated).
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_outlier_seam.py -v`
Expected: all PASS (existing name-based tests confirm no regression)

- [ ] **Step 5: Commit**

```bash
git add bindings/util/tdoa_replay.py test_python/test_tdoa_outlier_seam.py
git commit -m "feat: replay accepts outlier-filter instances, not just core names"
```

---

### Task 2: Move experimental policies/filters to `tools/tdoa_plot/experiments.py`

**Files:**
- Create: `tools/tdoa_plot/experiments.py`
- Modify: `bindings/util/tdoa_selection.py` (remove `MedianPolicy`, `RoundRobinPolicy`; rename `_measurement` → `to_estimator_measurement`; shrink registry; docstrings)
- Modify: `bindings/util/tdoa_outlier.py` (remove `SanityFilter`, `MadWindowFilter`, `PairIntegratorFilter`; rename `_is_physically_possible` → `is_physically_possible`, `_anchor_distance_sq` → `anchor_distance_sq`; shrink registry; docstrings)
- Modify: `tools/tdoa_plot/plot_tdoa.py` (resolve policy/filter via experiments)
- Modify: `tools/usdlog/replay_tdoa.py` (same)
- Create: `test_python/test_tdoa_experiments.py`
- Modify: `test_python/test_tdoa_outlier.py`, `test_python/test_tdoa_selection.py`, `test_python/test_tdoa_outlier_seam.py`

**Interfaces:**
- Consumes: instance-accepting `replay` (Task 1).
- Produces:
  - `bindings.util.tdoa_selection`: `POLICY_FACTORIES == {'baseline', 'all'}`, `to_estimator_measurement(group, candidate) -> dict` (public), `make_policy(name)` unchanged otherwise.
  - `bindings.util.tdoa_outlier`: `FILTER_FACTORIES == {'integrator', 'none'}`, `is_physically_possible(tdoa) -> bool`, `anchor_distance_sq(tdoa) -> float` (public).
  - `tools.tdoa_plot.experiments`: classes `MedianPolicy`, `RoundRobinPolicy`, `SanityFilter`, `MadWindowFilter`, `PairIntegratorFilter`; dicts `EXPERIMENTAL_POLICY_FACTORIES`, `EXPERIMENTAL_FILTER_FACTORIES`; `resolve_policy(name) -> SelectionPolicy`, `resolve_outlier_filter(name) -> OutlierFilter` (merge experimental over core; `ValueError` listing all names on unknown).

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_experiments.py`:

```python
"""Tests for the experimental (analysis-side) policies and outlier filters.

These classes are research payload, not exposure: they live above the cut
line in tools/tdoa_plot and are resolved by name via resolve_policy /
resolve_outlier_filter, merged over the core registries in bindings/util.
No cffirmware bindings required.
"""

import pytest

from tools.tdoa_plot.experiments import (
    EXPERIMENTAL_FILTER_FACTORIES,
    EXPERIMENTAL_POLICY_FACTORIES,
    resolve_outlier_filter,
    resolve_policy,
)


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


def _group(candidates, idA=1, t_ms=100.0):
    """Candidate group in the build_candidate_groups schema."""
    return {'group': 0, 't_ms': t_ms, 'idA': idA,
            'candidates': [
                {'idB': idB, 'distanceDiff': dd, 'isSelected': bool(sel)}
                for idB, dd, sel in candidates]}


def test_experimental_registries_hold_the_moved_classes():
    assert set(EXPERIMENTAL_POLICY_FACTORIES) == {'median', 'round_robin'}
    assert set(EXPERIMENTAL_FILTER_FACTORIES) == {
        'sanity', 'mad_window', 'pair_integrator'}


def test_resolver_finds_core_and_experimental_names():
    assert resolve_policy('baseline').name == 'baseline'
    assert resolve_policy('median').name == 'median'
    assert resolve_outlier_filter('none').name == 'none'
    assert resolve_outlier_filter('mad_window').name == 'mad_window'


def test_resolver_unknown_name_lists_core_and_experimental():
    with pytest.raises(ValueError) as e:
        resolve_policy('nope')
    assert 'baseline' in str(e.value) and 'median' in str(e.value)
    with pytest.raises(ValueError) as e:
        resolve_outlier_filter('nope')
    assert 'integrator' in str(e.value) and 'mad_window' in str(e.value)


def test_resolvers_return_fresh_instances():
    assert resolve_outlier_filter('mad_window') is not resolve_outlier_filter('mad_window')
    assert resolve_policy('round_robin') is not resolve_policy('round_robin')


def test_median_policy_picks_candidate_closest_to_group_median():
    group = _group([(2, 0.5, 0), (3, 0.6, 1), (4, 5.0, 0)])
    m = resolve_policy('median').select(group)
    assert len(m) == 1
    # Median of (0.5, 0.6, 5.0) is 0.6 -> candidate idB=3; estimator
    # convention flips ids: idA = remote (3), idB = packet anchor (1).
    assert m[0] == {'idA': 3, 'idB': 1, 'distanceDiff': 0.6}
```

Also MOVE these test functions verbatim (delete from source file, add to `test_python/test_tdoa_experiments.py`, adapting only the factory call as shown):

- From `test_python/test_tdoa_selection.py`: `test_round_robin_rotates_and_resets` — replace `make_policy('round_robin')` with `resolve_policy('round_robin')` and drop the now-unused import if any.
- From `test_python/test_tdoa_outlier.py` (they use the `_Tdoa`/`_Vec` stubs already duplicated above): `test_sanity_rejects_physically_impossible_distance_diff`, `test_mad_window_accepts_everything_during_warmup`, `test_mad_window_rejects_far_from_median_after_warmup`, `test_mad_window_rejected_samples_still_enter_the_window`, `test_pair_integrator_isolates_anchor_pairs`, `test_pair_integrator_reset_reopens_all_pairs`, `test_pair_integrator_out_of_order_timestamp_clamps_like_c_uint32` — replace every `make_outlier_filter('<name>')` with `resolve_outlier_filter('<name>')`.

Then update the shrunken-registry tests in place:

In `test_python/test_tdoa_outlier.py`, replace `test_all_spec_filters_are_registered`, `test_unknown_name_raises_value_error_listing_available` and `test_factories_return_fresh_instances` with:

```python
def test_core_registry_holds_parity_and_reference_filters_only():
    # Experimental filters live in tools/tdoa_plot/experiments.py (above the
    # cut line); the core registry is parity ('integrator') + reference ('none').
    assert set(FILTER_FACTORIES) == {'integrator', 'none'}


def test_unknown_name_raises_value_error_listing_available():
    with pytest.raises(ValueError, match='integrator'):
        make_outlier_filter('nope')


def test_factories_return_fresh_instances():
    assert make_outlier_filter('none') is not make_outlier_filter('none')
```

In `test_python/test_tdoa_selection.py`, add (next to the existing policy tests):

```python
def test_core_registry_holds_parity_and_reference_policies_only():
    from bindings.util.tdoa_selection import POLICY_FACTORIES
    assert set(POLICY_FACTORIES) == {'baseline', 'all'}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_experiments.py test_python/test_tdoa_outlier.py test_python/test_tdoa_selection.py -v`
Expected: `test_tdoa_experiments.py` errors with `ModuleNotFoundError: No module named 'tools.tdoa_plot.experiments'`; the two registry tests FAIL (registries still hold 5/4 entries)

- [ ] **Step 3: Create `tools/tdoa_plot/experiments.py`**

Move the class bodies verbatim from their current locations (`MedianPolicy`: `bindings/util/tdoa_selection.py:117-133`, `RoundRobinPolicy`: `:135-155`; `SanityFilter`: `bindings/util/tdoa_outlier.py:73-78`, `MadWindowFilter`: `:81-123`, `PairIntegratorFilter`: `:125-174`), adapting only the names imported from `bindings.util` (public helper names, `to_estimator_measurement`). Full file:

```python
"""
Experimental TDoA selection policies and outlier filters (analysis side).

These are candidate tdoa3 improvements — hypotheses under test, not exposure
of firmware behavior — so they live above the exposure/analysis cut line
(see docs/superpowers/specs/2026-07-08-tdoa-analysis-views-design.md).
The mechanisms they plug into (SelectionPolicy, OutlierFilter) and the
firmware-parity baselines stay below the line in bindings/util. An
experiment that wins graduates to the firmware; one that loses is deleted.

resolve_policy / resolve_outlier_filter merge these experimental registries
over the core ones, so CLIs accept both sets of names. Filters are passed to
``replay`` as instances (the seam only resolves core names itself).
"""

from collections import deque

from bindings.util.tdoa_outlier import (
    FILTER_FACTORIES,
    OutlierFilter,
    is_physically_possible,
)
from bindings.util.tdoa_selection import (
    POLICY_FACTORIES,
    SelectionPolicy,
    to_estimator_measurement,
)


class MedianPolicy(SelectionPolicy):
    """Pick the single candidate whose distanceDiff is closest to the group median.

    A cheap consensus/outlier-resistant selector: an outlier candidate far from
    the rest of the group is unlikely to be chosen.
    """
    name = 'median'

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        vals = sorted(c['distanceDiff'] for c in cs)
        median = vals[len(vals) // 2]
        best = min(cs, key=lambda c: abs(c['distanceDiff'] - median))
        return [to_estimator_measurement(group, best)]


class RoundRobinPolicy(SelectionPolicy):
    """Pick one candidate per packet, rotating through the group indices.

    Mimics the spirit of the firmware's rotating ("Random") matcher, but replayed
    deterministically so results are reproducible.
    """
    name = 'round_robin'

    def __init__(self):
        self._k = 0

    def reset(self):
        self._k = 0

    def select(self, group):
        cs = group['candidates']
        if not cs:
            return []
        c = cs[self._k % len(cs)]
        self._k += 1
        return [to_estimator_measurement(group, c)]


class SanityFilter(OutlierFilter):
    """Only the physically-impossible check; no innovation gating."""
    name = 'sanity'

    def validate(self, tdoa, error, now_ms):
        return is_physically_possible(tdoa)


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
            accepted = abs(error - med) < self.K * mad
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
        if not is_physically_possible(tdoa):
            return False
        key = (int(tdoa.anchorIdA), int(tdoa.anchorIdB))
        s = self._pairs.setdefault(
            key, {'integrator': 0.0, 'is_open': True, 'latest_ms': 0})

        accepted_distance = tdoa.stdDev * self.ACCEPT_STD_MULTIPLIER
        trigger_distance = tdoa.stdDev * self.TRIGGER_STD_MULTIPLIER

        dt_ms = now_ms - s['latest_ms']
        if dt_ms < 0:
            # The C original computes this in uint32_t: a negative delta wraps
            # to a huge value and the fminf clamp reduces it to the cap.
            dt_ms = self.INTEGRATOR_SIZE / 10.0
        else:
            dt_ms = min(dt_ms, self.INTEGRATOR_SIZE / 10.0)
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


EXPERIMENTAL_POLICY_FACTORIES = {
    'median': MedianPolicy,
    'round_robin': RoundRobinPolicy,
}

EXPERIMENTAL_FILTER_FACTORIES = {
    'sanity': SanityFilter,
    'mad_window': MadWindowFilter,
    'pair_integrator': PairIntegratorFilter,
}


def resolve_policy(name):
    """Instantiate a selection policy by name, core or experimental."""
    factories = {**POLICY_FACTORIES, **EXPERIMENTAL_POLICY_FACTORIES}
    try:
        return factories[name]()
    except KeyError:
        raise ValueError(
            f"Unknown selection policy '{name}'. "
            f"Available: {', '.join(sorted(factories))}")


def resolve_outlier_filter(name):
    """Instantiate an outlier filter by name, core or experimental."""
    factories = {**FILTER_FACTORIES, **EXPERIMENTAL_FILTER_FACTORIES}
    try:
        return factories[name]()
    except KeyError:
        raise ValueError(
            f"Unknown outlier filter '{name}'. "
            f"Available: {', '.join(sorted(factories))}")
```

- [ ] **Step 4: Shrink `bindings/util/tdoa_selection.py`**

- Delete `MedianPolicy` and `RoundRobinPolicy` classes.
- Rename `_measurement` to `to_estimator_measurement` (it is now public API used from above the line); update the two call sites in `BaselinePolicy.select` and `AllCandidatesPolicy.select`.
- Shrink the registry and update the error message:

```python
# Factories (not instances) so each replay run gets fresh policy state.
# Experimental policies live in tools/tdoa_plot/experiments.py (above the
# exposure/analysis cut line) and are resolved via resolve_policy there.
POLICY_FACTORIES = {
    'baseline': BaselinePolicy,
    'all': AllCandidatesPolicy,
}


def make_policy(name):
    try:
        return POLICY_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown selection policy '{name}'. "
            f"Available: {', '.join(sorted(POLICY_FACTORIES))} "
            f"(experimental policies: tools/tdoa_plot/experiments.py)")
```

- Append to the `AllCandidatesPolicy` docstring:

```
    Kept below the cut line as the trivial reference example of the
    plug-point contract: a policy may return 0, 1 or many measurements.
```

(`BaselinePolicy`'s docstring already states it reproduces what the live firmware did — leave as is.)

- [ ] **Step 5: Shrink `bindings/util/tdoa_outlier.py`**

- Delete `SanityFilter`, `MadWindowFilter`, `PairIntegratorFilter` and the now-unused `from collections import deque` import.
- Rename `_anchor_distance_sq` → `anchor_distance_sq` and `_is_physically_possible` → `is_physically_possible` (public firmware-mirroring utilities; the docstring of `is_physically_possible` already cites `outlierFilterTdoa.c`).
- Shrink the registry and update the error message:

```python
# Factories (not instances) so each replay run gets fresh filter state.
# Experimental filters live in tools/tdoa_plot/experiments.py (above the
# exposure/analysis cut line) and are resolved via resolve_outlier_filter there.
FILTER_FACTORIES = {
    'integrator': IntegratorFilter,
    'none': NoneFilter,
}


def make_outlier_filter(name):
    try:
        return FILTER_FACTORIES[name]()
    except KeyError:
        raise ValueError(
            f"Unknown outlier filter '{name}'. "
            f"Available: {', '.join(sorted(FILTER_FACTORIES))} "
            f"(experimental filters: tools/tdoa_plot/experiments.py)")
```

- Update the `NoneFilter` docstring:

```python
class NoneFilter(OutlierFilter):
    """Accept everything — the control group.

    Kept below the cut line as the trivial reference example of the
    plug-point contract.
    """
```

(`IntegratorFilter`'s docstring already says "The real firmware filter … the firmware-parity baseline" — leave as is.)

- [ ] **Step 6: Update the CLIs to use the resolvers**

`tools/tdoa_plot/plot_tdoa.py`, in `load_series`:

```python
    from tools.tdoa_plot.experiments import resolve_outlier_filter, resolve_policy
```

(replace the `make_policy` import from `tdoa_selection`), then:

```python
    policy = resolve_policy(args.selection_policy)
```

and pass the filter as an instance:

```python
    outlier_filter = (resolve_outlier_filter(args.outlier_filter)
                      if args.outlier_filter else None)
    replayed = replay(anchor_positions, imu_samples, tdoa_samples,
                      {'tdoa_std': args.tdoa_std,
                       'tdoa_model': args.tdoa_model,
                       'outlier_filter': outlier_filter})
```

Update both `--selection-policy` and `--outlier-filter` help texts to reference `tools/tdoa_plot/experiments.py` for the experimental names.

`tools/usdlog/replay_tdoa.py`: replace the `make_policy` import with `from tools.tdoa_plot.experiments import resolve_outlier_filter, resolve_policy`, use `resolve_policy(name)` in the main loop, and change `run_policy` to take a filter instance:

```python
            policy = resolve_policy(name)
            trajectory = run_policy(policy, anchor_positions, imu_samples,
                                    groups, args.tdoa_std,
                                    resolve_outlier_filter(filter_name),
                                    args.tdoa_model)
```

(`run_policy` itself is unchanged — it just forwards `outlier_filter` into the params dict, which now carries an instance.)

- [ ] **Step 7: Update the seam test to cover both registries**

In `test_python/test_tdoa_outlier_seam.py`:

```python
def test_every_registered_filter_replays_and_converges():
    from bindings.util.tdoa_outlier import FILTER_FACTORIES
    from tools.tdoa_plot.experiments import (
        EXPERIMENTAL_FILTER_FACTORIES, resolve_outlier_filter)
    imu, tdoa = _synthetic_imu(), _synthetic_tdoa_with_outliers()
    for name in list(FILTER_FACTORIES) + list(EXPERIMENTAL_FILTER_FACTORIES):
        trajectory = replay(_anchor_positions(), list(imu), list(tdoa),
                            {'tdoa_std': 0.15,
                             'outlier_filter': resolve_outlier_filter(name)})
        assert len(trajectory) > 4000, name
        _, final_pos = trajectory[-1]
        # Outliers are injected, so allow a loose bound; 'none' is the worst.
        assert _dist(final_pos, TRUE_POS) < 1.5, name
```

and in `test_gated_robust_model_replays_and_converges` replace `'outlier_filter': 'mad_window'` with:

```python
    from tools.tdoa_plot.experiments import resolve_outlier_filter
    ...
                         'outlier_filter': resolve_outlier_filter('mad_window')})
```

- [ ] **Step 8: Run the full suite**

Run: `make test_python`
Expected: all PASS (moved tests run from `test_tdoa_experiments.py`; registry tests see the shrunken registries; CLIs are exercised by `test_tdoa_plot.py` import)

- [ ] **Step 9: Verify CLI behavior is unchanged**

Run: `cd tools/tdoa_plot && uv run plot_tdoa.py ../../run_validation.bin --anchors ../../anchors.yaml --selection-policy median --outlier-filter mad_window --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/task2_check.png`
Expected: replays and saves the figure without error (experimental names still work end to end)

- [ ] **Step 10: Commit**

```bash
git add tools/tdoa_plot/experiments.py bindings/util/tdoa_selection.py \
    bindings/util/tdoa_outlier.py tools/tdoa_plot/plot_tdoa.py \
    tools/usdlog/replay_tdoa.py test_python/test_tdoa_experiments.py \
    test_python/test_tdoa_outlier.py test_python/test_tdoa_selection.py \
    test_python/test_tdoa_outlier_seam.py
git commit -m "refactor: move experimental policies/filters above the cut line"
```

---

### Task 3: Create `bindings/util/tdoa_truth.py` (ground-truth extraction moves down)

**Files:**
- Create: `bindings/util/tdoa_truth.py`
- Modify: `tools/usdlog/replay_tdoa.py` (import from tdoa_truth; delete moved functions)
- Modify: `tools/tdoa_plot/plot_tdoa.py:65,100,120` (imports)
- Create: `test_python/test_tdoa_truth.py`
- Modify: `test_python/test_tdoa_replay.py` (retarget interp test)

**Interfaces:**
- Produces: `bindings.util.tdoa_truth.extract_ground_truth(log_data) -> [(t_ms, (x, y, z))]` and `interp_ground_truth(gt, t_ms) -> (x, y, z) | None` (public name, no underscore). Task 9 extends this module; the views plan consumes it.

- [ ] **Step 1: Write the failing test**

Create `test_python/test_tdoa_truth.py`:

```python
"""Tests for ground-truth math (no cffirmware bindings required)."""

from bindings.util.tdoa_truth import extract_ground_truth, interp_ground_truth


def test_extract_ground_truth_skips_origin_samples_and_sorts():
    log = {'fixedFrequency': {
        'timestamp': [20.0, 10.0, 30.0],
        'lighthouse.x': [0.2, 0.1, 0.0],
        'lighthouse.y': [0.0, 0.0, 0.0],
        'lighthouse.z': [1.0, 1.0, 0.0],
    }}
    gt = extract_ground_truth(log)
    # The (0, 0, 0) sample is the firmware's no-crossing-beam marker: dropped.
    assert gt == [(10.0, (0.1, 0.0, 1.0)), (20.0, (0.2, 0.0, 1.0))]


def test_extract_ground_truth_handles_missing_block():
    assert extract_ground_truth({}) == []
    assert extract_ground_truth({'fixedFrequency': {'timestamp': [1.0]}}) == []


def test_interp_ground_truth_interpolates_and_bounds():
    gt = [(100.0, (0.0, 0.0, 0.0)), (200.0, (1.0, 0.0, 0.0))]
    assert interp_ground_truth(gt, 150.0) == (0.5, 0.0, 0.0)
    assert interp_ground_truth(gt, 100.0) == (0.0, 0.0, 0.0)
    assert interp_ground_truth(gt, 99.0) is None
    assert interp_ground_truth(gt, 201.0) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_truth.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'bindings.util.tdoa_truth'`

- [ ] **Step 3: Create the module**

Create `bindings/util/tdoa_truth.py`. `extract_ground_truth` moves verbatim from `tools/usdlog/replay_tdoa.py:43-66`; `interp_ground_truth` is `_interp_ground_truth` from `:78-95` with the public name:

```python
"""
Lighthouse ground-truth math for TDoA candidate logs.

The logs carry the Lighthouse crossing-beam position (ground truth) and the
anchor positions are known, so a *true* distance difference — and therefore a
ground-truth measurement error — is computable for every logged candidate,
independent of the Kalman filter. This module is the below-the-cut-line home
for that math: extraction, interpolation and (from phase 2 of the design)
per-measurement truth labeling.

Pure Python; must stay importable without the cffirmware bindings. Anchor
positions are plain (x, y, z) triples here, not cffirmware vec3 objects.
"""
```

then the two moved functions (docstrings unchanged).

- [ ] **Step 4: Update the consumers**

- `tools/usdlog/replay_tdoa.py`: delete `extract_ground_truth` and `_interp_ground_truth`; add `from bindings.util.tdoa_truth import extract_ground_truth, interp_ground_truth` and rename the two internal uses (`score_trajectory`, `write_csv`) from `_interp_ground_truth` to `interp_ground_truth`.
- `tools/tdoa_plot/plot_tdoa.py`: replace `from tools.usdlog.replay_tdoa import extract_ground_truth` (line 65) and `from tools.usdlog.replay_tdoa import _interp_ground_truth` (line 100) with imports from `bindings.util.tdoa_truth`; rename the `_interp_ground_truth` call in `print_stats` to `interp_ground_truth`.
- `test_python/test_tdoa_replay.py`: in `test_ground_truth_interpolation_and_scoring`, drop the interpolation asserts (now covered by `test_tdoa_truth.py`), rename the test to `test_score_trajectory`, and import `score_trajectory` only:

```python
def test_score_trajectory():
    # Scoring lives in the CLI module; importing it must not require the
    # cffirmware bindings.
    from tools.usdlog.replay_tdoa import score_trajectory

    gt = [(100.0, (0.0, 0.0, 0.0)), (200.0, (1.0, 0.0, 0.0))]

    # Trajectory exactly on the ground truth scores zero error
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 0.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['n'] == 2
    assert metrics['rms'] == 0.0
    assert metrics['flyaway_frac'] == 0.0

    # A 1 m error on one of two samples is reflected in max and flyaway_frac
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 1.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['max'] == 1.0
    assert metrics['flyaway_frac'] == 0.5
```

- [ ] **Step 5: Run the full suite**

Run: `make test_python`
Expected: all PASS

- [ ] **Step 6: Commit**

```bash
git add bindings/util/tdoa_truth.py tools/usdlog/replay_tdoa.py \
    tools/tdoa_plot/plot_tdoa.py test_python/test_tdoa_truth.py \
    test_python/test_tdoa_replay.py
git commit -m "refactor: ground-truth extraction/interpolation moves to bindings/util/tdoa_truth"
```

---

### Task 4: Dissolve `replay_tdoa.py` into `analyze_tdoa.py compare`

**Files:**
- Create: `tools/tdoa_plot/analyze_tdoa.py`
- Delete: `tools/usdlog/replay_tdoa.py`
- Modify: `tools/usdlog/README_tdoa_candidates.md` (lines 31, 114, 174, 179, 209 reference `replay_tdoa`)
- Modify: `test_python/test_tdoa_replay.py` (retarget `score_trajectory` import)

**Interfaces:**
- Consumes: `resolve_policy`/`resolve_outlier_filter` (Task 2), `tdoa_truth` (Task 3), `ensure_bindings` from `tools.tdoa_plot.plot_tdoa`.
- Produces: `tools/tdoa_plot/analyze_tdoa.py` — argparse CLI with subparsers; subcommand `compare` (the relocated policy × filter × model table). `score_trajectory(trajectory, gt, flyaway_threshold_m) -> (metrics_dict, errors_list)` and `write_csv(path, trajectory, gt)` live at module level (the views plan and tests import them). Module import must not require cffirmware (lazy imports, same discipline as `plot_tdoa.py`).

- [ ] **Step 1: Write the failing test**

In `test_python/test_tdoa_replay.py`, change the import in `test_score_trajectory` to:

```python
    from tools.tdoa_plot.analyze_tdoa import score_trajectory
```

and update its comment to say the scoring lives in the analysis CLI module.

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_replay.py::test_score_trajectory -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tools.tdoa_plot.analyze_tdoa'`

- [ ] **Step 3: Create `tools/tdoa_plot/analyze_tdoa.py`**

The `compare` implementation is `replay_tdoa.py`'s `main()` body, `run_policy`, `score_trajectory` and `write_csv` moved verbatim except: resolvers from `experiments`, truth imports from `tdoa_truth`, and argparse restructured into subcommands. Skeleton (moved function bodies elided here only to avoid duplication — copy them verbatim from `tools/usdlog/replay_tdoa.py` as committed in Task 3):

```python
"""
TDoA analysis views over a Crazyflie uSD candidate log.

Subcommands answer specific analysis questions (see
docs/superpowers/specs/2026-07-08-tdoa-analysis-views-design.md, Q1-Q8).
Currently implemented:

    compare   Replay policy x filter x model combinations and score each
              against the Lighthouse ground truth (Q8).

Run from this folder with uv::

    uv run analyze_tdoa.py compare ../../run_validation.bin --anchors ../../anchors.yaml

or from the repository root::

    uv run --project tools/tdoa_plot tools/tdoa_plot/analyze_tdoa.py compare \
        run_validation.bin --anchors anchors.yaml
"""

import argparse
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'build'))
sys.path.insert(0, str(REPO_ROOT))


def run_policy(policy, anchor_positions, imu_samples, groups, tdoa_std,
               outlier_filter, tdoa_model):
    # moved verbatim from tools/usdlog/replay_tdoa.py (lazy-imports
    # apply_policy/replay from bindings.util.tdoa_replay inside the function
    # body so importing this module never needs the bindings)
    from bindings.util.tdoa_replay import apply_policy, replay
    tdoa_samples = apply_policy(policy, groups)
    return replay(anchor_positions, imu_samples, tdoa_samples,
                  {'tdoa_std': tdoa_std, 'outlier_filter': outlier_filter,
                   'tdoa_model': tdoa_model})


def score_trajectory(trajectory, gt, flyaway_threshold_m):
    # moved verbatim (uses interp_ground_truth from bindings.util.tdoa_truth,
    # imported at module top — tdoa_truth is pure Python, so this keeps the
    # module importable without the bindings)
    ...


def write_csv(path, trajectory, gt):
    # moved verbatim
    ...


def cmd_compare(args):
    # replay_tdoa.py main() body moved verbatim, with:
    #   make_policy(name)          -> resolve_policy(name)
    #   filter_name into params    -> resolve_outlier_filter(filter_name)
    # imports of cfusdlog/tdoa_replay/tdoa_selection/loco_utils/experiments
    # done lazily here, after ensure_bindings()
    from tools.tdoa_plot.plot_tdoa import ensure_bindings
    ensure_bindings(rebuild=args.rebuild_bindings)
    ...


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('compare',
                       help='Score policy x filter x model combinations '
                            'against ground truth (Q8)')
    p.add_argument('logfile', help='uSD log file (binary) recorded on the Crazyflie')
    p.add_argument('--anchors', required=True,
                   help='YAML file mapping anchor id -> {x, y, z}')
    p.add_argument('--policies', nargs='+',
                   default=['baseline', 'all', 'median', 'round_robin'],
                   help='Selection policies to evaluate')
    p.add_argument('--outlier-filters', nargs='+', default=['integrator'],
                   help='Outlier filters to evaluate')
    p.add_argument('--tdoa-model', choices=['standard', 'robust'], default='standard',
                   help='TDoA measurement model for the whole run')
    p.add_argument('--tdoa-std', type=float, default=0.15,
                   help='TDoA measurement std dev [m] used in the Kalman update')
    p.add_argument('--flyaway-threshold', type=float, default=0.3,
                   help='Position error [m] above which a sample counts as a fly-away')
    p.add_argument('--csv-dir', default=None,
                   help='If set, write a per-policy trajectory+error CSV here')
    p.add_argument('--rebuild-bindings', action='store_true',
                   help='Force a rebuild of the cffirmware bindings')
    p.set_defaults(func=cmd_compare)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
```

Keep the baseline-reconstruction report, the baseline-vs-live divergence line, and the summary table formatting exactly as they are in `replay_tdoa.py`'s `main()`.

Then `git rm tools/usdlog/replay_tdoa.py`.

- [ ] **Step 4: Update `tools/usdlog/README_tdoa_candidates.md`**

Replace every `python3 -m tools.usdlog.replay_tdoa ...` invocation with the `uv run --project tools/tdoa_plot tools/tdoa_plot/analyze_tdoa.py compare ...` equivalent, the bullet on line 31 with `analyze_tdoa.py compare (tools/tdoa_plot) — replay + score policies vs ground truth`, and the `replay_tdoa.py --tdoa-std` reference on line 209 with `analyze_tdoa.py compare --tdoa-std`.

- [ ] **Step 5: Run the full suite**

Run: `make test_python`
Expected: all PASS. Also confirm no stale references: `grep -rn "replay_tdoa" --include="*.py" bindings/ tools/ test_python/` returns nothing.

- [ ] **Step 6: Verify the CLI end to end**

Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py compare ../../run_validation.bin --anchors ../../anchors.yaml --policies baseline --outlier-filters integrator`
Expected: the log summary, baseline-reconstruction OK line and scoring table print, matching what `replay_tdoa.py` printed before

- [ ] **Step 7: Commit**

```bash
git add tools/tdoa_plot/analyze_tdoa.py tools/usdlog/README_tdoa_candidates.md \
    test_python/test_tdoa_replay.py
git rm tools/usdlog/replay_tdoa.py
git commit -m "refactor: dissolve replay_tdoa into analyze_tdoa compare subcommand"
```

---

### Task 5: Covariance element getter (`cffirmware.i`)

**Files:**
- Modify: `bindings/cffirmware.i` (the `%{ %}` header block and the `%inline` block)
- Create: `test_python/test_kalman_binding_helpers.py`

**Interfaces:**
- Produces: `cffirmware.kalmanCoreGetCovarianceElement(coreData, i, j) -> float` — `P[i][j]`, NaN outside `[0, KC_STATE_DIM)`. Tasks 6, 8 and the views plan consume it (with `cffirmware.KC_STATE_X/Y/Z`, already exposed).

- [ ] **Step 1: Write the failing test**

Create `test_python/test_kalman_binding_helpers.py`:

```python
"""Tests for the analysis binding helpers in cffirmware.i / mm_tdoa.c."""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')


def _default_core():
    params = cffirmware.kalmanCoreParams_t()
    cffirmware.kalmanCoreDefaultParams(params)
    core = cffirmware.kalmanCoreData_t()
    cffirmware.kalmanCoreInit(core, params, 0)
    return core, params


def test_covariance_diagonal_matches_default_initial_variances():
    core, params = _default_core()
    get = cffirmware.kalmanCoreGetCovarianceElement
    assert get(core, cffirmware.KC_STATE_X, cffirmware.KC_STATE_X) == pytest.approx(
        params.stdDevInitialPosition_xy ** 2)
    assert get(core, cffirmware.KC_STATE_Y, cffirmware.KC_STATE_Y) == pytest.approx(
        params.stdDevInitialPosition_xy ** 2)
    assert get(core, cffirmware.KC_STATE_Z, cffirmware.KC_STATE_Z) == pytest.approx(
        params.stdDevInitialPosition_z ** 2)


def test_covariance_is_diagonal_after_init():
    core, _ = _default_core()
    get = cffirmware.kalmanCoreGetCovarianceElement
    dim = cffirmware.KC_STATE_DIM
    for i in range(dim):
        for j in range(dim):
            if i != j:
                assert get(core, i, j) == 0.0, (i, j)


def test_covariance_out_of_bounds_returns_nan():
    core, _ = _default_core()
    get = cffirmware.kalmanCoreGetCovarianceElement
    dim = cffirmware.KC_STATE_DIM
    for i, j in [(-1, 0), (0, -1), (dim, 0), (0, dim)]:
        assert math.isnan(get(core, i, j)), (i, j)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_binding_helpers.py -v`
Expected: FAIL with `AttributeError: module 'cffirmware' has no attribute 'kalmanCoreGetCovarianceElement'`

- [ ] **Step 3: Implement**

In `bindings/cffirmware.i`, add `#include <math.h>` to the `%{ ... %}` header block (for `NAN`), and append inside the existing `%inline %{ ... %}` block:

```c
// Read access to the Kalman covariance matrix for offline analysis
// (kalmanCoreData_t.P is a 2D float array, which SWIG wraps as an opaque
// object). Index with the kalmanCoreStateIdx_t enum (KC_STATE_X, ...).
// Returns NAN when an index is outside [0, KC_STATE_DIM).
float kalmanCoreGetCovarianceElement(const kalmanCoreData_t* coreData, int i, int j)
{
    if (i < 0 || i >= KC_STATE_DIM || j < 0 || j >= KC_STATE_DIM) {
        return NAN;
    }
    return coreData->P[i][j];
}
```

- [ ] **Step 4: Rebuild bindings and run the tests**

Run: `make bindings_python && PYTHONPATH=build python3 -m pytest test_python/test_kalman_binding_helpers.py -v`
Expected: 3 PASS

- [ ] **Step 5: Commit**

```bash
git add bindings/cffirmware.i test_python/test_kalman_binding_helpers.py
git commit -m "feat: expose Kalman covariance elements to the Python bindings"
```

---

### Task 6: `kalmanCoreTdoaInnovationVariance` (mm_tdoa.c)

**Files:**
- Modify: `src/modules/src/kalman_core/mm_tdoa.c` (after `kalmanCoreTdoaInnovation`, line 118)
- Modify: `src/modules/interface/kalman_core/mm_tdoa.h` (after the `kalmanCoreTdoaInnovation` declaration)
- Test: `test_python/test_kalman_binding_helpers.py`

**Interfaces:**
- Consumes: covariance getter (Task 5) for the test's expected value.
- Produces: `cffirmware.kalmanCoreTdoaInnovationVariance(coreData, tdoa) -> float` — S = HPHᵀ + R (H the 1×`KC_STATE_DIM` measurement row vector from the TDoA measurement model, R = `stdDev`²); NaN exactly when `kalmanCoreTdoaInnovation` is NaN. Task 8 and the views plan (NIS) consume it.

- [ ] **Step 1: Write the failing tests**

Add to `test_python/test_kalman_binding_helpers.py`:

```python
def _tdoa(pa, pb, dd=0.0, std=0.15):
    t = cffirmware.tdoaMeasurement_t()
    t.anchorIdA, t.anchorIdB = 0, 1
    pos_a, pos_b = cffirmware.point_t(), cffirmware.point_t()
    pos_a.x, pos_a.y, pos_a.z = pa
    pos_b.x, pos_b.y, pos_b.z = pb
    t.anchorPositionA, t.anchorPositionB = pos_a, pos_b
    t.distanceDiff = dd
    t.stdDev = std
    return t


def _expected_innovation_variance(core, params, pa, pb, std):
    # Mirror of measurementModel's jacobian (mm_tdoa.c) + S = h P h^T + R.
    # After kalmanCoreInit the state position is (initialX, initialY, initialZ).
    pos = (params.initialX, params.initialY, params.initialZ)
    d = [math.dist(pos, p) for p in (pa, pb)]
    h = {}
    for axis, idx in [(0, cffirmware.KC_STATE_X), (1, cffirmware.KC_STATE_Y),
                      (2, cffirmware.KC_STATE_Z)]:
        h[idx] = ((pos[axis] - pb[axis]) / d[1]) - ((pos[axis] - pa[axis]) / d[0])
    get = cffirmware.kalmanCoreGetCovarianceElement
    hph = sum(h[i] * get(core, i, j) * h[j] for i in h for j in h)
    return hph + std ** 2


def test_innovation_variance_matches_hand_computed_hph_plus_r():
    core, params = _default_core()
    pa, pb, std = (1.0, 2.0, 3.0), (4.0, 0.0, 0.0), 0.15
    s = cffirmware.kalmanCoreTdoaInnovationVariance(core, _tdoa(pa, pb, std=std))
    assert s == pytest.approx(
        _expected_innovation_variance(core, params, pa, pb, std), rel=1e-5)


def test_innovation_variance_is_at_least_the_measurement_variance():
    core, _ = _default_core()
    for pa, pb in [((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                   ((5.0, 5.0, 5.0), (-5.0, 5.0, 0.0)),
                   ((0.0, 0.0, 2.0), (0.0, 3.0, 2.0))]:
        s = cffirmware.kalmanCoreTdoaInnovationVariance(core, _tdoa(pa, pb))
        assert s >= 0.15 ** 2 - 1e-9, (pa, pb)


def test_innovation_variance_nan_exactly_when_innovation_is_nan():
    core, params = _default_core()
    # Anchor A at the state position -> degenerate (d0 == 0) in both helpers.
    degenerate = _tdoa((params.initialX, params.initialY, params.initialZ),
                       (4.0, 0.0, 0.0))
    assert math.isnan(cffirmware.kalmanCoreTdoaInnovation(core, degenerate))
    assert math.isnan(cffirmware.kalmanCoreTdoaInnovationVariance(core, degenerate))
```

Note: the sum in `_expected_innovation_variance` only covers the position rows/columns of P — correct here because h is zero outside X/Y/Z.

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_kalman_binding_helpers.py -v -k innovation_variance`
Expected: FAIL with `AttributeError: ... no attribute 'kalmanCoreTdoaInnovationVariance'`

- [ ] **Step 3: Implement**

`src/modules/interface/kalman_core/mm_tdoa.h`, after the `kalmanCoreTdoaInnovation` declaration:

```c
// Innovation variance S = H P H^T + R of a TDoA measurement against the
// current state (H the 1 x KC_STATE_DIM measurement jacobian, R = stdDev^2),
// without updating anything. Returns NAN in the degenerate case, mirroring
// kalmanCoreTdoaInnovation. Companion analysis helper for the Python
// bindings (NIS / filter-consistency checks); not used by the flight code.
float kalmanCoreTdoaInnovationVariance(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa);
```

`src/modules/src/kalman_core/mm_tdoa.c`, after `kalmanCoreTdoaInnovation` (line 118):

```c
float kalmanCoreTdoaInnovationVariance(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return NAN;
  }
  float hph = 0.0f;
  for (int i = 0; i < KC_STATE_DIM; i++) {
    float ph = 0.0f;
    for (int j = 0; j < KC_STATE_DIM; j++) {
      ph += this->P[i][j] * h[j];
    }
    hph += h[i] * ph;
  }
  return hph + tdoa->stdDev * tdoa->stdDev;
}
```

- [ ] **Step 4: Rebuild bindings and run the tests**

Run: `make bindings_python && PYTHONPATH=build python3 -m pytest test_python/test_kalman_binding_helpers.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add src/modules/src/kalman_core/mm_tdoa.c \
    src/modules/interface/kalman_core/mm_tdoa.h \
    test_python/test_kalman_binding_helpers.py
git commit -m "feat: kalmanCoreTdoaInnovationVariance analysis helper"
```

---

### Task 7: Outlier-filter `snapshot()`

**Files:**
- Modify: `bindings/util/tdoa_outlier.py` (base class + `IntegratorFilter`)
- Modify: `tools/tdoa_plot/experiments.py` (`MadWindowFilter`, `PairIntegratorFilter`)
- Test: `test_python/test_tdoa_outlier.py`, `test_python/test_tdoa_outlier_seam.py`, `test_python/test_tdoa_experiments.py`

**Interfaces:**
- Produces: `OutlierFilter.snapshot() -> dict` — small, JSON-serializable view of internal state; base returns `{}`. `IntegratorFilter`: `{'integrator': float, 'is_open': bool}`. `PairIntegratorFilter`: `{'pair': (idA, idB) | None, 'integrator': float, 'is_open': bool}` for the most recently validated pair. `MadWindowFilter`: `{'n': int, 'median': float | None, 'mad': float | None}`. Task 8's recorder calls `snapshot()` after each `validate()`.

- [ ] **Step 1: Write the failing tests**

In `test_python/test_tdoa_outlier.py`:

```python
def test_snapshot_default_is_empty_dict():
    assert make_outlier_filter('none').snapshot() == {}
```

In `test_python/test_tdoa_outlier_seam.py`:

```python
def test_integrator_snapshot_reflects_c_state():
    from bindings.util.tdoa_outlier import IntegratorFilter
    f = IntegratorFilter()
    snap = f.snapshot()
    assert set(snap) == {'integrator', 'is_open'}
    assert isinstance(snap['integrator'], float)
    assert isinstance(snap['is_open'], bool)
    # A good sample raises the integrator level (dt capped in C).
    tdoa = cffirmware.tdoaMeasurement_t()
    tdoa.anchorIdA, tdoa.anchorIdB = 0, 1
    a, b = cffirmware.point_t(), cffirmware.point_t()
    a.x, a.y, a.z = 0.0, 0.0, 0.0
    b.x, b.y, b.z = 4.0, 0.0, 0.0
    tdoa.anchorPositionA, tdoa.anchorPositionB = a, b
    tdoa.distanceDiff = 0.0
    tdoa.stdDev = 0.15
    f.validate(tdoa, error=0.0, now_ms=100)
    assert f.snapshot()['integrator'] > snap['integrator']
```

In `test_python/test_tdoa_experiments.py`:

```python
def test_pair_integrator_snapshot_tracks_last_validated_pair():
    f = resolve_outlier_filter('pair_integrator')
    assert f.snapshot() == {'pair': None, 'integrator': 0.0, 'is_open': True}
    f.validate(_Tdoa(ids=(3, 4)), error=0.0, now_ms=100)
    snap = f.snapshot()
    assert snap['pair'] == (3, 4)
    assert snap['integrator'] > 0.0
    assert snap['is_open'] is True


def test_mad_window_snapshot_reports_window_stats():
    f = resolve_outlier_filter('mad_window')
    assert f.snapshot() == {'n': 0, 'median': None, 'mad': None}
    for i in range(25):
        f.validate(_Tdoa(), error=float(i % 2), now_ms=i)
    snap = f.snapshot()
    assert snap['n'] == 25
    assert snap['median'] is not None and snap['mad'] is not None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_outlier.py::test_snapshot_default_is_empty_dict test_python/test_tdoa_outlier_seam.py::test_integrator_snapshot_reflects_c_state test_python/test_tdoa_experiments.py -v`
Expected: snapshot tests FAIL with `AttributeError: ... 'snapshot'`

- [ ] **Step 3: Implement**

`bindings/util/tdoa_outlier.py`, in the base class:

```python
    def snapshot(self):
        """Small, JSON-serializable view of the internal state, recorded per
        sample by the replay diagnostics (ReplayRecorder). Base: stateless."""
        return {}
```

`IntegratorFilter`:

```python
    def snapshot(self):
        return {'integrator': float(self._state.integrator),
                'is_open': bool(self._state.isFilterOpen)}
```

`tools/tdoa_plot/experiments.py` — `PairIntegratorFilter`: in `reset()` add `self._last_key = None`; at the end of `validate()` (just before `return`) add `self._last_key = key`; then:

```python
    def snapshot(self):
        s = self._pairs.get(self._last_key)
        if s is None:
            return {'pair': None, 'integrator': 0.0, 'is_open': True}
        return {'pair': self._last_key,
                'integrator': s['integrator'], 'is_open': s['is_open']}
```

`MadWindowFilter`:

```python
    def snapshot(self):
        if not self._window:
            return {'n': 0, 'median': None, 'mad': None}
        med = self._median(self._window)
        return {'n': len(self._window), 'median': med,
                'mad': self._median(abs(e - med) for e in self._window)}
```

- [ ] **Step 4: Run the full suite**

Run: `make test_python`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add bindings/util/tdoa_outlier.py tools/tdoa_plot/experiments.py \
    test_python/test_tdoa_outlier.py test_python/test_tdoa_outlier_seam.py \
    test_python/test_tdoa_experiments.py
git commit -m "feat: outlier-filter snapshot() for replay diagnostics"
```

---

### Task 8: `ReplayRecorder` + emulator/replay instrumentation

**Files:**
- Modify: `bindings/util/tdoa_replay.py` (`ReplayRecorder` class; `replay()` gains `recorder=None`)
- Modify: `bindings/util/estimator_kalman_emulator.py` (`recorder` param; record in the TDoA path and per iteration)
- Create: `test_python/test_tdoa_recorder.py`

**Interfaces:**
- Consumes: `kalmanCoreGetCovarianceElement` (Task 5), `kalmanCoreTdoaInnovationVariance` (Task 6), `snapshot()` (Task 7).
- Produces:
  - `ReplayRecorder` (in `bindings.util.tdoa_replay`): attributes `tdoa_records: list[dict]`, `state_records: list[dict]`, `n_skipped_unknown_anchor: int`; methods `record_tdoa(record: dict)`, `record_state(record: dict)`.
  - `tdoa_records` entry keys: `t_ms, idA, idB, distanceDiff, stdDev, innovation, innovation_var, accepted, filter_state` (`accepted: True | False | None`; `filter_state: dict` from `snapshot()` taken after the filter processed the sample; for `accepted is None` the snapshot is taken without a `validate` call — the firmware skips degenerate samples before the filter).
  - `state_records` entry keys: `t_ms, x, y, z, var_x, var_y, var_z`; one entry per 1 kHz iteration, same `t_ms` as the trajectory entry.
  - `replay(anchor_positions, imu_samples, tdoa_samples, params=None, recorder=None)`.
  - `EstimatorKalmanEmulator(anchor_positions, tdoa_model='standard', outlier_filter=None, recorder=None)`; raises `ValueError` when `recorder` is set but `outlier_filter` is `None`.

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_recorder.py`:

```python
"""Tests for ReplayRecorder diagnostics (needs the cffirmware bindings)."""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')

from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator
from bindings.util.tdoa_outlier import IntegratorFilter, NoneFilter
from bindings.util.tdoa_replay import ReplayRecorder, replay

ANCHOR_COORDS = {
    0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (0.0, 4.0, 0.0), 3: (4.0, 4.0, 0.0),
    4: (0.0, 0.0, 3.0), 5: (4.0, 0.0, 3.0), 6: (0.0, 4.0, 3.0), 7: (4.0, 4.0, 3.0),
}
TRUE_POS = (2.0, 2.0, 1.0)
DURATION_MS = 2000


def _anchor_positions(ids=None):
    result = {}
    for anchor_id, (x, y, z) in ANCHOR_COORDS.items():
        if ids is not None and anchor_id not in ids:
            continue
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


def _synthetic_tdoa():
    samples = []
    for k, t in enumerate(range(0, DURATION_MS, 10)):
        id_a, id_b = k % 8, (k + 1) % 8
        dd = (_dist(ANCHOR_COORDS[id_b], TRUE_POS)
              - _dist(ANCHOR_COORDS[id_a], TRUE_POS))
        samples.append(('estTDOA',
                        {'timestamp': float(t), 'idA': id_a, 'idB': id_b,
                         'distanceDiff': dd}))
    return samples


def _recorded_replay(outlier_filter, tdoa=None):
    recorder = ReplayRecorder()
    trajectory = replay(_anchor_positions(), _synthetic_imu(),
                        tdoa if tdoa is not None else _synthetic_tdoa(),
                        {'tdoa_std': 0.15, 'outlier_filter': outlier_filter},
                        recorder=recorder)
    return trajectory, recorder


def test_recorder_requires_an_externalized_outlier_filter():
    # The built-in C path hides the accept/reject decision, so diagnostics
    # without an externalized filter would silently lie. Fail fast instead.
    with pytest.raises(ValueError, match='outlier'):
        EstimatorKalmanEmulator(_anchor_positions(), recorder=ReplayRecorder())
    with pytest.raises(ValueError, match='outlier'):
        replay(_anchor_positions(), _synthetic_imu()[:10], [],
               {}, recorder=ReplayRecorder())


def test_one_tdoa_record_per_sample_with_expected_keys():
    tdoa = _synthetic_tdoa()
    trajectory, recorder = _recorded_replay(NoneFilter(), tdoa)
    assert len(recorder.tdoa_records) == len(tdoa)
    r = recorder.tdoa_records[0]
    assert set(r) == {'t_ms', 'idA', 'idB', 'distanceDiff', 'stdDev',
                      'innovation', 'innovation_var', 'accepted', 'filter_state'}
    # NoneFilter accepts everything and the geometry is non-degenerate.
    assert all(r['accepted'] is True for r in recorder.tdoa_records)


def test_state_records_align_with_trajectory():
    trajectory, recorder = _recorded_replay(NoneFilter())
    assert len(recorder.state_records) == len(trajectory)
    for (t_ms, pos), rec in zip(trajectory, recorder.state_records):
        assert rec['t_ms'] == t_ms
        assert rec['x'] == pos[0] and rec['y'] == pos[1] and rec['z'] == pos[2]
    assert all(rec['var_x'] > 0.0 and rec['var_y'] > 0.0 and rec['var_z'] > 0.0
               for rec in recorder.state_records)


def test_innovation_variance_is_pre_update_and_at_least_r():
    _, recorder = _recorded_replay(NoneFilter())
    assert all(r['innovation_var'] >= 0.15 ** 2 - 1e-9
               for r in recorder.tdoa_records)


def test_integrator_filter_state_is_recorded():
    _, recorder = _recorded_replay(IntegratorFilter())
    assert all(set(r['filter_state']) == {'integrator', 'is_open'}
               for r in recorder.tdoa_records)
    # Clean synthetic data: the integrator level must rise over the run.
    assert (recorder.tdoa_records[-1]['filter_state']['integrator']
            > recorder.tdoa_records[0]['filter_state']['integrator'])


def test_degenerate_geometry_records_accepted_none():
    # Anchor 0 sits at the origin = the initial state position, and the first
    # sample arrives before any prediction has moved the state: d0 == 0 in the
    # measurement model -> NaN innovation -> firmware skips before the filter.
    tdoa = [('estTDOA', {'timestamp': 0.0, 'idA': 0, 'idB': 1,
                         'distanceDiff': 0.0})]
    _, recorder = _recorded_replay(NoneFilter(), tdoa)
    r = recorder.tdoa_records[0]
    assert math.isnan(r['innovation'])
    assert r['accepted'] is None


def test_skipped_unknown_anchor_count_lands_on_the_recorder():
    tdoa = _synthetic_tdoa()
    recorder = ReplayRecorder()
    # Drop anchor 7 from the anchor file: samples touching it are skipped.
    n_touching_7 = sum(1 for s in tdoa if 7 in (s[1]['idA'], s[1]['idB']))
    replay(_anchor_positions(ids=range(7)), _synthetic_imu(), tdoa,
           {'tdoa_std': 0.15, 'outlier_filter': NoneFilter()},
           recorder=recorder)
    assert recorder.n_skipped_unknown_anchor == n_touching_7
    assert len(recorder.tdoa_records) == len(tdoa) - n_touching_7
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_recorder.py -v`
Expected: FAIL with `ImportError: cannot import name 'ReplayRecorder'`

- [ ] **Step 3: Implement `ReplayRecorder` in `bindings/util/tdoa_replay.py`**

```python
class ReplayRecorder:
    """Collects per-measurement and per-iteration diagnostics during replay().

    Records are plain dicts (JSON-serializable) so analysis tooling — which
    lives above the exposure/analysis cut line — can consume or dump them
    without depending on anything in this module.

    tdoa_records: one dict per TDoA sample reaching the estimator, with keys
        t_ms, idA, idB, distanceDiff, stdDev, innovation, innovation_var,
        accepted (True/False from the outlier filter, None when the innovation
        is NaN and the firmware skips the sample before the filter), and
        filter_state (the filter's snapshot() after processing the sample).
    state_records: one dict per 1 kHz iteration, with keys
        t_ms, x, y, z, var_x, var_y, var_z (position variance from the
        Kalman covariance matrix).
    n_skipped_unknown_anchor: TDoA samples dropped before the replay because
        an anchor id was missing from the anchors file.
    """

    def __init__(self):
        self.tdoa_records = []
        self.state_records = []
        self.n_skipped_unknown_anchor = 0

    def record_tdoa(self, record):
        self.tdoa_records.append(record)

    def record_state(self, record):
        self.state_records.append(record)
```

Update `replay()` — signature `def replay(anchor_positions, imu_samples, tdoa_samples, params=None, recorder=None):`, docstring gains:

```
        recorder: optional ReplayRecorder collecting per-measurement and
            per-iteration diagnostics. Requires an externalized
            'outlier_filter' in params (the built-in C path hides the
            accept/reject decision); the emulator raises ValueError otherwise.
```

then pass it through and record the skip count:

```python
    emulator = EstimatorKalmanEmulator(
        anchor_positions, tdoa_model=params.get('tdoa_model', 'standard'),
        outlier_filter=outlier_filter, recorder=recorder)
    ...
    tdoa_samples, n_skipped = filter_known_anchors(tdoa_samples, anchor_positions)
    if recorder is not None:
        recorder.n_skipped_unknown_anchor = n_skipped
```

- [ ] **Step 4: Instrument `bindings/util/estimator_kalman_emulator.py`**

Constructor:

```python
    def __init__(self, anchor_positions, tdoa_model='standard',
                 outlier_filter=None, recorder=None) -> None:
        if tdoa_model not in TDOA_MODELS:
            raise ValueError(
                f"unknown tdoa_model '{tdoa_model}', expected one of {TDOA_MODELS}")
        if recorder is not None and outlier_filter is None:
            raise ValueError(
                'diagnostics recording requires an externalized outlier filter: '
                'the built-in C path hides the accept/reject decision '
                "(use outlier_filter='integrator' for firmware parity)")
        self.recorder = recorder
```

(rest unchanged). Docstring: mention the optional recorder next to the existing outlier_filter paragraph.

In `_update_with_sample`, replace the externalized-filter branch:

```python
            else:
                error = cffirmware.kalmanCoreTdoaInnovation(self.coreData, tdoa)
                if math.isnan(error):
                    # Degenerate geometry; firmware skips the sample without
                    # consulting the filter, so neither do we.
                    accepted = None
                else:
                    accepted = bool(self.outlier_filter.validate(tdoa, error, now_ms))
                if self.recorder is not None:
                    # innovation_var is computed BEFORE the update (pre-update
                    # S, what NIS needs), like the innovation itself.
                    self.recorder.record_tdoa({
                        't_ms': now_ms,
                        'idA': int(tdoa.anchorIdA),
                        'idB': int(tdoa.anchorIdB),
                        'distanceDiff': float(tdoa.distanceDiff),
                        'stdDev': float(tdoa.stdDev),
                        'innovation': float(error),
                        'innovation_var': float(
                            cffirmware.kalmanCoreTdoaInnovationVariance(
                                self.coreData, tdoa)),
                        'accepted': accepted,
                        'filter_state': self.outlier_filter.snapshot(),
                    })
                if accepted:
                    if self.tdoa_model == 'robust':
                        cffirmware.kalmanCoreRobustUpdateWithTdoa(
                            self.coreData, tdoa, self.outlierFilterState)
                    else:
                        cffirmware.kalmanCoreUpdateWithTdoaUnfiltered(self.coreData, tdoa)
```

Note the recording happens before the update (state and covariance still pre-update) and `if accepted:` treats `None` as falsy, preserving today's behavior exactly when no recorder is attached.

In `run_one_1khz_iteration`, before the `return`:

```python
        if self.recorder is not None:
            get = cffirmware.kalmanCoreGetCovarianceElement
            self.recorder.record_state({
                't_ms': self.now_ms,
                'x': external_state.position.x,
                'y': external_state.position.y,
                'z': external_state.position.z,
                'var_x': get(self.coreData, cffirmware.KC_STATE_X, cffirmware.KC_STATE_X),
                'var_y': get(self.coreData, cffirmware.KC_STATE_Y, cffirmware.KC_STATE_Y),
                'var_z': get(self.coreData, cffirmware.KC_STATE_Z, cffirmware.KC_STATE_Z),
            })
```

- [ ] **Step 5: Run the tests**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_recorder.py -v`
Expected: 7 PASS

- [ ] **Step 6: Run the full suite (recorder must not disturb parity)**

Run: `make test_python`
Expected: all PASS — in particular `test_integrator_via_seam_is_bit_identical_to_legacy_path` (no recorder attached: behavior byte-identical)

- [ ] **Step 7: Commit**

```bash
git add bindings/util/tdoa_replay.py bindings/util/estimator_kalman_emulator.py \
    test_python/test_tdoa_recorder.py
git commit -m "feat: ReplayRecorder diagnostics on the replay seam"
```

---

### Task 9: Truth labeling (`tdoa_truth.py` phase-2 extension)

**Files:**
- Modify: `bindings/util/tdoa_truth.py`
- Test: `test_python/test_tdoa_truth.py`

**Interfaces:**
- Consumes: `interp_ground_truth` (Task 3); candidate-group schema from `tdoa_selection.build_candidate_groups`; `to_estimator_measurement` (Task 2).
- Produces (all pure Python, positions are `(x, y, z)` triples):
  - `true_distance_diff(pos, pos_a, pos_b) -> float` — `d(pos, pos_b) - d(pos, pos_a)`, the estimator-convention truth.
  - `as_position_tuples(anchor_positions) -> dict[int, tuple]` — adapter from objects with `.x/.y/.z` (e.g. `loco_utils` vec3) to plain triples.
  - `flatten_candidate_groups(groups) -> list[dict]` — one row per candidate in the estimator convention: `t_ms, idA, idB, distanceDiff, isSelected, group`.
  - `label_measurements(rows, gt, anchor_positions) -> list[dict]` — copies of the input rows (any dicts with `t_ms/idA/idB/distanceDiff`, so `tdoa_records` work directly) with `true_diff` and `true_error = distanceDiff - true_diff` added; both `None` when `t_ms` is outside the GT range or an anchor id is unknown.
  - `is_outlier(true_error, threshold) -> bool` — `False` for `None`.

- [ ] **Step 1: Write the failing tests**

Add to `test_python/test_tdoa_truth.py`:

```python
from bindings.util.tdoa_truth import (
    as_position_tuples,
    flatten_candidate_groups,
    is_outlier,
    label_measurements,
    true_distance_diff,
)


def test_true_distance_diff_exact_values():
    a, b = (0.0, 0.0, 0.0), (4.0, 0.0, 0.0)
    assert true_distance_diff((2.0, 0.0, 0.0), a, b) == 0.0
    # d(B) = 3, d(A) = 1 -> estimator-convention diff = 2
    assert true_distance_diff((1.0, 0.0, 0.0), a, b) == 2.0
    assert true_distance_diff((3.0, 4.0, 0.0), (0.0, 0.0, 0.0), (3.0, 0.0, 0.0)) \
        == 4.0 - 5.0


def test_flatten_candidate_groups_uses_estimator_convention():
    groups = [{'group': 7, 't_ms': 100.0, 'idA': 1,
               'candidates': [
                   {'idB': 2, 'distanceDiff': 0.5, 'isSelected': True},
                   {'idB': 3, 'distanceDiff': 0.6, 'isSelected': False}]}]
    rows = flatten_candidate_groups(groups)
    # Estimator convention: idA = remote candidate, idB = packet anchor —
    # same flip as to_estimator_measurement / the live enqueueTDOA.
    assert rows == [
        {'t_ms': 100.0, 'idA': 2, 'idB': 1, 'distanceDiff': 0.5,
         'isSelected': True, 'group': 7},
        {'t_ms': 100.0, 'idA': 3, 'idB': 1, 'distanceDiff': 0.6,
         'isSelected': False, 'group': 7},
    ]


def test_label_measurements_adds_true_diff_and_error():
    anchors = {1: (0.0, 0.0, 0.0), 2: (4.0, 0.0, 0.0)}
    # Tag truly at (1, 0, 0) for the whole GT range: true_diff for
    # (idA=1, idB=2) is d(B) - d(A) = 3 - 1 = 2.
    gt = [(100.0, (1.0, 0.0, 0.0)), (200.0, (1.0, 0.0, 0.0))]
    rows = [
        {'t_ms': 150.0, 'idA': 1, 'idB': 2, 'distanceDiff': 2.25},
        {'t_ms': 999.0, 'idA': 1, 'idB': 2, 'distanceDiff': 2.0},   # out of GT range
        {'t_ms': 150.0, 'idA': 1, 'idB': 9, 'distanceDiff': 2.0},   # unknown anchor
    ]
    labeled = label_measurements(rows, gt, anchors)
    assert labeled[0]['true_diff'] == pytest.approx(2.0)
    assert labeled[0]['true_error'] == pytest.approx(0.25)
    assert labeled[1]['true_diff'] is None and labeled[1]['true_error'] is None
    assert labeled[2]['true_diff'] is None and labeled[2]['true_error'] is None
    # Inputs are not mutated.
    assert 'true_diff' not in rows[0]


def test_is_outlier_thresholds_and_handles_none():
    assert is_outlier(0.5, threshold=0.3)
    assert is_outlier(-0.5, threshold=0.3)
    assert not is_outlier(0.2, threshold=0.3)
    assert not is_outlier(None, threshold=0.3)


def test_as_position_tuples_adapts_objects_with_xyz():
    class V:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    assert as_position_tuples({3: V(1.0, 2.0, 3.0)}) == {3: (1.0, 2.0, 3.0)}
```

Add `import pytest` at the top of the file (for `pytest.approx`).

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_truth.py -v`
Expected: new tests FAIL with `ImportError`

- [ ] **Step 3: Implement in `bindings/util/tdoa_truth.py`**

```python
import math

from bindings.util.tdoa_selection import to_estimator_measurement


def true_distance_diff(pos, pos_a, pos_b):
    """Ground-truth distanceDiff at position pos, estimator convention.

    The estimator measurement (see to_estimator_measurement) carries
    distanceDiff = d(idB) - d(idA); this is that quantity computed from a
    known position: |pos - pos_b| - |pos - pos_a|.
    """
    return math.dist(pos, pos_b) - math.dist(pos, pos_a)


def as_position_tuples(anchor_positions):
    """Adapt a dict of objects with .x/.y/.z (e.g. loco_utils vec3) to the
    plain (x, y, z) triples this module works with."""
    return {k: (float(v.x), float(v.y), float(v.z))
            for k, v in anchor_positions.items()}


def flatten_candidate_groups(groups):
    """One row per candidate, in the estimator convention, truth-labelable.

    Unlike apply_policy output this keeps isSelected and the group id, so
    selection quality (Q2) is analyzable per candidate.
    """
    rows = []
    for group in groups:
        for candidate in group['candidates']:
            row = to_estimator_measurement(group, candidate)
            row['t_ms'] = group['t_ms']
            row['isSelected'] = candidate['isSelected']
            row['group'] = group['group']
            rows.append(row)
    return rows


def label_measurements(rows, gt, anchor_positions):
    """Join ground truth onto measurement rows (copies; inputs untouched).

    Each row needs t_ms/idA/idB/distanceDiff — flatten_candidate_groups rows
    and ReplayRecorder tdoa_records both qualify. Adds true_diff and
    true_error = distanceDiff - true_diff; both None when t_ms is outside the
    ground-truth range or an anchor id is not in anchor_positions.
    """
    labeled = []
    for row in rows:
        row = dict(row)
        pos = interp_ground_truth(gt, row['t_ms'])
        pos_a = anchor_positions.get(row['idA'])
        pos_b = anchor_positions.get(row['idB'])
        if pos is None or pos_a is None or pos_b is None:
            row['true_diff'] = None
            row['true_error'] = None
        else:
            row['true_diff'] = true_distance_diff(pos, pos_a, pos_b)
            row['true_error'] = row['distanceDiff'] - row['true_diff']
        labeled.append(row)
    return labeled


def is_outlier(true_error, threshold):
    """Ground-truth outlier label; None (unlabelable) is not an outlier."""
    return true_error is not None and abs(true_error) > threshold
```

(`import math` merges with the module's existing imports; `interp_ground_truth` is already defined above in the same module.)

- [ ] **Step 4: Run the full suite**

Run: `make test_python`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add bindings/util/tdoa_truth.py test_python/test_tdoa_truth.py
git commit -m "feat: ground-truth labeling of TDoA measurements (tdoa_truth phase 2)"
```

---

### Task 10: Data-layer validation on the real run

**Files:**
- None modified (validation only; fix anything it surfaces)

- [ ] **Step 1: Full suite green**

Run: `make test_python`
Expected: all PASS

- [ ] **Step 2: Recorded replay on `run_validation.bin`**

Run from repo root:

```bash
PYTHONPATH=build python3 - <<'EOF'
import sys
sys.path.insert(0, '.')
import tools.usdlog.cfusdlog as cfusdlog
from bindings.util.loco_utils import read_loco_anchor_positions
from bindings.util.tdoa_outlier import IntegratorFilter
from bindings.util.tdoa_replay import ReplayRecorder, apply_policy, extract_imu_samples, replay
from bindings.util.tdoa_selection import build_candidate_groups, make_policy
from bindings.util.tdoa_truth import (
    as_position_tuples, extract_ground_truth, flatten_candidate_groups,
    is_outlier, label_measurements)

log = cfusdlog.decode('run_validation.bin')
anchors = read_loco_anchor_positions('anchors.yaml')
groups = build_candidate_groups(log)
recorder = ReplayRecorder()
traj = replay(anchors, extract_imu_samples(log),
              apply_policy(make_policy('baseline'), groups),
              {'tdoa_std': 0.15, 'outlier_filter': IntegratorFilter()},
              recorder=recorder)
gt = extract_ground_truth(log)
labeled = label_measurements(recorder.tdoa_records, gt, as_position_tuples(anchors))
n_labeled = sum(1 for r in labeled if r['true_error'] is not None)
n_out = sum(1 for r in labeled if is_outlier(r['true_error'], 0.5))
n_rej = sum(1 for r in labeled if r['accepted'] is False)
cand = label_measurements(flatten_candidate_groups(groups), gt, as_position_tuples(anchors))
print(f'trajectory: {len(traj)} iterations, state_records: {len(recorder.state_records)}')
print(f'tdoa_records: {len(recorder.tdoa_records)}, labeled: {n_labeled}, '
      f'|true_error|>0.5m: {n_out}, rejected: {n_rej}, '
      f'skipped unknown-anchor: {recorder.n_skipped_unknown_anchor}')
print(f'candidate rows: {len(cand)}')
print('first record:', {k: v for k, v in recorder.tdoa_records[0].items()})
EOF
```

Expected: runs to completion; `state_records` count equals trajectory count; most records labeled; rejected count plausible (nonzero but a small fraction); the first record shows all nine keys with finite innovation/innovation_var.

- [ ] **Step 3: Sanity-check the truth labels against the estimator**

The mean of `innovation - true_error` over accepted, labeled records should be near zero once converged (both measure the same measurement error, one against the estimate, one against GT). Eyeball from the step-2 script if in doubt — large systematic disagreement means a convention bug; stop and investigate before building views on top.

- [ ] **Step 4: Commit (only if fixes were needed)**

```bash
git add -A && git commit -m "fix: data-layer validation fixes from run_validation.bin"
```

---

## Execution notes

- Tasks must run in order: 1 → 2 → 3 → 4 (phase 0), 5 → 6 → 7 → 8 (phase 1), 9 → 10 (phase 2 + validation). Task 7 can run before 5/6; nothing else reorders.
- The views (phase 3) are a separate plan: `docs/superpowers/plans/2026-07-08-tdoa-analysis-views.md`.
