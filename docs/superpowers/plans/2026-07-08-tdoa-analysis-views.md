# TDoA Analysis Views Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement phase 3 of `docs/superpowers/specs/2026-07-08-tdoa-analysis-views-design.md`: the eight analysis subcommands of `tools/tdoa_plot/analyze_tdoa.py` (Q1–Q8), built on the data layer delivered by `2026-07-08-tdoa-diagnostics-data-layer.md`.

**Architecture:** Everything in this plan lives above the exposure/analysis cut line, in `tools/tdoa_plot/`. Shared glue (`common.py`) loads a run and produces recorded replays; pure numeric analysis (`analysis.py`) computes confusion matrices, regret, NIS, DOP etc. and is unit-tested exactly; each view module (`views/*.py`) turns those numbers into one matplotlib figure; `analyze_tdoa.py` wires subcommands. Views consume only the three documented data products (candidate groups, `ReplayRecorder` records, truth labels).

**Tech Stack:** Python 3.10+, matplotlib + numpy (present in the `tools/tdoa_plot` uv venv), pytest (`test_python/`, plot tests behind `pytest.importorskip('matplotlib')` with the Agg backend).

## Global Constraints

- **Prerequisite:** the data-layer plan (`2026-07-08-tdoa-diagnostics-data-layer.md`) is fully executed. This plan consumes: `ReplayRecorder` (`tdoa_records`/`state_records`/`n_skipped_unknown_anchor`), `tdoa_truth` (`extract_ground_truth`, `interp_ground_truth`, `true_distance_diff`, `as_position_tuples`, `flatten_candidate_groups`, `label_measurements`, `is_outlier`), `experiments` (`resolve_policy`, `resolve_outlier_filter`), `analyze_tdoa.py` with the `compare` subcommand, `snapshot()` filter state.
- Nothing under `bindings/` or `src/` is modified by this plan.
- Chart rules (dataviz): **no dual/twin axes** — related series of different scale get their own subplot row sharing the x-axis; **no color-cycling** over an unbounded set (per-anchor plots use small multiples); accept/reject is encoded by color **and** marker shape; every multi-series axes has a legend; grids stay recessive (`alpha=0.3`, matching `plot_tdoa.py`).
- Categorical palette (validated colorblind-safe, in `common.py`): `#0173B2` (blue), `#DE8F05` (orange), `#029E73` (green), `#D55E00` (vermillion), `#CC78BC` (pink). Accepted = blue, rejected = vermillion, ground truth = black, live estimate = orange, replay = blue.
- Time axes are seconds (`t_ms / 1000.0`), labeled `time [s]`, like `plot_tdoa.py`.
- Tests: `PYTHONPATH=build python3 -m pytest test_python/<file> -v`; full suite `make test_python` green at the end of every task.
- Every figure-producing test uses:
  ```python
  matplotlib = pytest.importorskip('matplotlib')
  matplotlib.use('Agg')
  ```
  before importing the view module, and saves to `tmp_path` to prove the figure renders.

---

### Task 1: Shared glue (`tools/tdoa_plot/common.py`)

**Files:**
- Create: `tools/tdoa_plot/common.py`
- Modify: `tools/tdoa_plot/plot_tdoa.py` (`ensure_bindings` moves out; import from `common`)
- Modify: `tools/tdoa_plot/analyze_tdoa.py` (`ensure_bindings` import from `common`)
- Test: `test_python/test_tdoa_common.py`

**Interfaces:**
- Produces (all consumed by later tasks):
  - `common.ensure_bindings(rebuild=False)` — moved verbatim from `plot_tdoa.py:34-51`.
  - `common.load_run(logfile, anchors_path) -> dict` with keys `log_data, groups, gt, live, imu, anchor_positions` (cffirmware vec3 dict), `anchor_tuples` (plain triples).
  - `common.recorded_replay(run, selection_policy='baseline', outlier_filter='integrator', tdoa_model='standard', tdoa_std=0.15) -> (trajectory, recorder)`.
  - `common.save_or_show(fig, save_path)` — savefig at dpi 150 + print, or `plt.show()`.
  - Palette constants: `CATEGORICAL` (the 5-color list above), `COLOR_ACCEPTED`, `COLOR_REJECTED`, `COLOR_GT`, `COLOR_LIVE`, `COLOR_REPLAY`.
- Module import must not require cffirmware or matplotlib (lazy imports inside functions).

- [ ] **Step 1: Write the failing test**

Create `test_python/test_tdoa_common.py`:

```python
"""Tests for the analysis-CLI glue (no bindings, no matplotlib needed)."""


def test_common_imports_without_bindings_or_matplotlib():
    import tools.tdoa_plot.common as common
    assert callable(common.ensure_bindings)
    assert callable(common.load_run)
    assert callable(common.recorded_replay)
    assert callable(common.save_or_show)


def test_palette_constants():
    import tools.tdoa_plot.common as common
    assert len(common.CATEGORICAL) == 5
    assert common.COLOR_ACCEPTED != common.COLOR_REJECTED
    for c in common.CATEGORICAL:
        assert c.startswith('#') and len(c) == 7


def test_plot_tdoa_reexports_ensure_bindings():
    # analyze_tdoa/plot_tdoa CLIs both bootstrap through common.
    from tools.tdoa_plot.common import ensure_bindings
    from tools.tdoa_plot.plot_tdoa import ensure_bindings as via_plot
    assert via_plot is ensure_bindings
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_common.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tools.tdoa_plot.common'`

- [ ] **Step 3: Create `tools/tdoa_plot/common.py`**

```python
"""
Shared glue for the tdoa_plot analysis CLIs (analyze_tdoa.py, plot_tdoa.py).

Everything here is analysis-side (above the exposure/analysis cut line) and
consumes only the documented data products of bindings/util: candidate
groups, ReplayRecorder diagnostics and tdoa_truth labels.
"""

import importlib
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
# Repo modules (tools.usdlog, bindings.util) and the bindings build output.
sys.path.insert(0, str(REPO_ROOT / 'build'))
sys.path.insert(0, str(REPO_ROOT))

# Colorblind-safe categorical palette (dataviz six-checks validated on a
# light surface; the orange/pink contrast WARN is relieved by legends on
# every view). Assign in fixed order, never cycle past 5 series.
CATEGORICAL = ['#0173B2', '#DE8F05', '#029E73', '#D55E00', '#CC78BC']
COLOR_ACCEPTED = '#0173B2'   # blue
COLOR_REJECTED = '#D55E00'   # vermillion
COLOR_GT = 'black'
COLOR_LIVE = '#DE8F05'       # orange
COLOR_REPLAY = '#0173B2'     # blue


def ensure_bindings(rebuild=False):
    """Import cffirmware, building it with make bindings_python if needed.

    The build is done with the *running* interpreter (PYTHON=sys.executable) so
    the extension matches the venv uv created, not whatever python3 is on PATH.
    """
    if not rebuild:
        try:
            import cffirmware  # noqa: F401
            return
        except ImportError as e:
            print(f'cffirmware bindings not importable ({e}); '
                  f'building with "make bindings_python" ...')
    subprocess.run(
        ['make', 'bindings_python', f'PYTHON={sys.executable}'],
        cwd=REPO_ROOT, check=True)
    importlib.invalidate_caches()
    import cffirmware  # noqa: F401


def load_run(logfile, anchors_path):
    """Decode a uSD log + anchors file into the dict every view consumes."""
    import tools.usdlog.cfusdlog as cfusdlog
    from bindings.util.loco_utils import read_loco_anchor_positions
    from bindings.util.tdoa_replay import (
        extract_imu_samples, extract_state_estimate)
    from bindings.util.tdoa_selection import build_candidate_groups
    from bindings.util.tdoa_truth import as_position_tuples, extract_ground_truth

    log_data = cfusdlog.decode(logfile)
    anchor_positions = read_loco_anchor_positions(anchors_path)
    return {
        'log_data': log_data,
        'groups': build_candidate_groups(log_data),
        'gt': extract_ground_truth(log_data),
        'live': extract_state_estimate(log_data),
        'imu': extract_imu_samples(log_data),
        'anchor_positions': anchor_positions,
        'anchor_tuples': as_position_tuples(anchor_positions),
    }


def recorded_replay(run, selection_policy='baseline', outlier_filter='integrator',
                    tdoa_model='standard', tdoa_std=0.15):
    """Replay the run with diagnostics; returns (trajectory, ReplayRecorder)."""
    from bindings.util.tdoa_replay import ReplayRecorder, apply_policy, replay
    from tools.tdoa_plot.experiments import (
        resolve_outlier_filter, resolve_policy)

    recorder = ReplayRecorder()
    tdoa = apply_policy(resolve_policy(selection_policy), run['groups'])
    trajectory = replay(
        run['anchor_positions'], list(run['imu']), tdoa,
        {'tdoa_std': tdoa_std, 'tdoa_model': tdoa_model,
         'outlier_filter': resolve_outlier_filter(outlier_filter)},
        recorder=recorder)
    return trajectory, recorder


def save_or_show(fig, save_path):
    import matplotlib.pyplot as plt

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f'Saved figure to {save_path}')
    else:
        plt.show()
```

- [ ] **Step 4: Update the two CLIs**

- `tools/tdoa_plot/plot_tdoa.py`: delete its `ensure_bindings` definition and the now-unused `importlib`/`subprocess` imports; add `from tools.tdoa_plot.common import ensure_bindings` below the existing `sys.path` bootstrap (the bootstrap block stays — it is what makes `tools.tdoa_plot.common` importable when the file runs as a script).
- `tools/tdoa_plot/analyze_tdoa.py`: in `cmd_compare`, replace `from tools.tdoa_plot.plot_tdoa import ensure_bindings` with `from tools.tdoa_plot.common import ensure_bindings`.

- [ ] **Step 5: Run the suite and the trajectory CLI**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run plot_tdoa.py ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/views_task1.png`
Expected: unchanged behavior

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/common.py tools/tdoa_plot/plot_tdoa.py \
    tools/tdoa_plot/analyze_tdoa.py test_python/test_tdoa_common.py
git commit -m "feat: shared analysis glue module (common.py)"
```

---

### Task 2: Numeric analysis core (`tools/tdoa_plot/analysis.py`)

**Files:**
- Create: `tools/tdoa_plot/analysis.py`
- Test: `test_python/test_tdoa_analysis.py`

**Interfaces:**
- Consumes: labeled rows (`tdoa_truth.label_measurements` output), candidate groups, `tdoa_records`, `state_records`, GT lists.
- Produces (each consumed by one or more views):
  - `confusion_matrix(labeled_records, threshold) -> dict` keys `tp, fp, fn, tn, unlabeled, undecided` — positive = ground-truth outlier; `tp` = rejected outlier, `fp` = rejected good, `fn` = accepted outlier, `tn` = accepted good.
  - `selection_regret(labeled_rows) -> list[dict]` keys `group, t_ms, selected_abs_error, best_abs_error, regret` — per candidate group with a labeled selection.
  - `packet_rate_per_anchor(groups, bin_ms=1000.0) -> dict[int, (centers_ms, rates_hz)]`.
  - `group_size_histogram(groups) -> dict[int, int]`.
  - `find_gaps(t_ms_list, min_gap_ms) -> list[(start_ms, end_ms)]`.
  - `acceptance_rate_series(tdoa_records, bin_ms=1000.0) -> (centers_ms, fractions)` — accepted/(decided) per bin; bins with no decisions get `None`.
  - `nis_series(tdoa_records) -> list[(t_ms, nis)]` — `innovation² / innovation_var`, finite entries only.
  - `consistency_series(state_records, gt) -> list[dict]` keys `t_ms, err_x, err_y, err_z, sigma_x, sigma_y, sigma_z` (GT-interpolatable iterations only).
  - `envelope_fraction(rows, k) -> dict` keys `x, y, z` — fraction of `|err| <= k * sigma`.
  - `tdoa_pdop(pairs, anchor_tuples, pos) -> float | None` — position DOP of the TDoA geometry (`None` for <3 usable pairs or singular geometry).
  - `dop_series(tdoa_records, gt, anchor_tuples, window_ms=1000.0) -> list[(center_ms, pdop | None)]` — from pairs *accepted* in each window.

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_analysis.py`:

```python
"""Exact-value tests for the numeric analysis core (no matplotlib needed)."""

import math

import pytest

from tools.tdoa_plot.analysis import (
    acceptance_rate_series,
    confusion_matrix,
    consistency_series,
    dop_series,
    envelope_fraction,
    find_gaps,
    group_size_histogram,
    nis_series,
    packet_rate_per_anchor,
    selection_regret,
    tdoa_pdop,
)


def _rec(t, accepted=True, true_error=0.0, innovation=0.1, innovation_var=0.04,
         idA=0, idB=1):
    return {'t_ms': t, 'idA': idA, 'idB': idB, 'distanceDiff': 0.0,
            'stdDev': 0.15, 'innovation': innovation,
            'innovation_var': innovation_var, 'accepted': accepted,
            'filter_state': {}, 'true_diff': 0.0, 'true_error': true_error}


def test_confusion_matrix_counts_all_quadrants():
    records = [
        _rec(0, accepted=False, true_error=1.0),    # rejected outlier: tp
        _rec(1, accepted=False, true_error=0.1),    # rejected good:    fp
        _rec(2, accepted=True, true_error=1.0),     # accepted outlier: fn
        _rec(3, accepted=True, true_error=0.1),     # accepted good:    tn
        _rec(4, accepted=True, true_error=None),    # unlabeled
        _rec(5, accepted=None, true_error=0.0),     # undecided (NaN innovation)
    ]
    assert confusion_matrix(records, threshold=0.5) == {
        'tp': 1, 'fp': 1, 'fn': 1, 'tn': 1, 'unlabeled': 1, 'undecided': 1}


def _crow(group, t, idA, idB, dd, selected, true_error):
    return {'group': group, 't_ms': t, 'idA': idA, 'idB': idB,
            'distanceDiff': dd, 'isSelected': selected,
            'true_diff': dd - true_error if true_error is not None else None,
            'true_error': true_error}


def test_selection_regret_selected_vs_oracle_best():
    rows = [
        _crow(0, 100.0, 2, 1, 0.5, True, 0.30),
        _crow(0, 100.0, 3, 1, 0.6, False, 0.10),   # oracle-best of group 0
        _crow(1, 110.0, 2, 4, 0.1, True, 0.05),    # selected == best
        _crow(1, 110.0, 3, 4, 0.2, False, -0.40),
        _crow(2, 120.0, 2, 5, 0.1, True, None),    # unlabeled selection: skipped
        _crow(3, 130.0, 2, 6, 0.1, False, 0.2),    # no selection: skipped
    ]
    out = selection_regret(rows)
    assert [r['group'] for r in out] == [0, 1]
    assert out[0]['selected_abs_error'] == pytest.approx(0.30)
    assert out[0]['best_abs_error'] == pytest.approx(0.10)
    assert out[0]['regret'] == pytest.approx(0.20)
    assert out[1]['regret'] == pytest.approx(0.0)


def _group(t_ms, idA, n_candidates):
    return {'group': int(t_ms), 't_ms': t_ms, 'idA': idA,
            'candidates': [{'idB': 10 + i, 'distanceDiff': 0.0,
                            'isSelected': i == 0} for i in range(n_candidates)]}


def test_packet_rate_per_anchor_bins_to_hz():
    groups = ([_group(float(t), 1, 2) for t in range(0, 1000, 100)]   # 10 in 1 s
              + [_group(float(t), 2, 2) for t in range(0, 1000, 200)])  # 5 in 1 s
    rates = packet_rate_per_anchor(groups, bin_ms=1000.0)
    assert set(rates) == {1, 2}
    centers, hz = rates[1]
    assert centers == [500.0]
    assert hz == [10.0]
    assert rates[2][1] == [5.0]


def test_group_size_histogram():
    groups = [_group(0.0, 1, 2), _group(1.0, 1, 2), _group(2.0, 1, 5)]
    assert group_size_histogram(groups) == {2: 2, 5: 1}


def test_find_gaps():
    ts = [0.0, 100.0, 2100.0, 2200.0]
    assert find_gaps(ts, min_gap_ms=1000.0) == [(100.0, 2100.0)]
    assert find_gaps(ts, min_gap_ms=3000.0) == []
    assert find_gaps([], min_gap_ms=1000.0) == []


def test_acceptance_rate_series_handles_empty_bins():
    records = ([_rec(float(t), accepted=True) for t in range(0, 1000, 100)]
               + [_rec(1050.0, accepted=False)]
               + [_rec(3050.0, accepted=True)])       # bin 2 (2000-3000): empty
    centers, fractions = acceptance_rate_series(records, bin_ms=1000.0)
    assert centers == [500.0, 1500.0, 2500.0, 3500.0]
    assert fractions[0] == pytest.approx(1.0)
    assert fractions[1] == pytest.approx(0.0)
    assert fractions[2] is None
    assert fractions[3] == pytest.approx(1.0)


def test_nis_series_skips_non_finite():
    records = [_rec(0.0, innovation=0.2, innovation_var=0.04),
               _rec(1.0, innovation=float('nan')),
               _rec(2.0, innovation=0.1, innovation_var=float('nan'))]
    out = nis_series(records)
    assert out == [(0.0, pytest.approx(1.0))]


def test_consistency_series_and_envelope_fraction():
    gt = [(0.0, (0.0, 0.0, 0.0)), (1000.0, (0.0, 0.0, 0.0))]
    state = [
        {'t_ms': 100.0, 'x': 0.1, 'y': 0.0, 'z': 0.0,
         'var_x': 0.04, 'var_y': 0.04, 'var_z': 0.04},
        {'t_ms': 200.0, 'x': 1.0, 'y': 0.0, 'z': 0.0,
         'var_x': 0.04, 'var_y': 0.04, 'var_z': 0.04},
        {'t_ms': 9999.0, 'x': 0.0, 'y': 0.0, 'z': 0.0,   # outside GT: dropped
         'var_x': 0.04, 'var_y': 0.04, 'var_z': 0.04},
    ]
    rows = consistency_series(state, gt)
    assert len(rows) == 2
    assert rows[0]['err_x'] == pytest.approx(0.1)
    assert rows[0]['sigma_x'] == pytest.approx(0.2)
    # k=3: |0.1| <= 0.6 in; |1.0| > 0.6 out -> x fraction 0.5, y/z 1.0
    frac = envelope_fraction(rows, k=3.0)
    assert frac['x'] == pytest.approx(0.5)
    assert frac['y'] == pytest.approx(1.0)


def test_tdoa_pdop_good_and_degenerate_geometry():
    pytest.importorskip('numpy')
    anchors = {0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0),
               2: (0.0, 4.0, 0.0), 4: (0.0, 0.0, 3.0)}
    pos = (2.0, 2.0, 1.0)
    good = tdoa_pdop([(0, 1), (0, 2), (0, 4)], anchors, pos)
    assert good is not None and good > 0.0
    # All pairs along x only: the y/z directions are unobservable -> singular.
    assert tdoa_pdop([(0, 1), (1, 0), (0, 1)], anchors, pos) is None
    # Fewer than 3 usable pairs (unknown anchor 9 drops one).
    assert tdoa_pdop([(0, 1), (0, 9)], anchors, pos) is None


def test_dop_series_uses_accepted_pairs_per_window():
    pytest.importorskip('numpy')
    anchors = {0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0),
               2: (0.0, 4.0, 0.0), 4: (0.0, 0.0, 3.0)}
    gt = [(0.0, (2.0, 2.0, 1.0)), (2000.0, (2.0, 2.0, 1.0))]
    records = [
        _rec(100.0, idA=0, idB=1), _rec(200.0, idA=0, idB=2),
        _rec(300.0, idA=0, idB=4),
        # Window 2: only one accepted pair -> None.
        _rec(1100.0, idA=0, idB=1),
        _rec(1200.0, idA=0, idB=2, accepted=False),
    ]
    out = dop_series(records, gt, anchors, window_ms=1000.0)
    assert out[0][0] == 500.0 and out[0][1] is not None
    assert out[1] == (1500.0, None)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_analysis.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tools.tdoa_plot.analysis'`

- [ ] **Step 3: Implement `tools/tdoa_plot/analysis.py`**

```python
"""
Numeric analysis over the TDoA data products (no plotting here).

Inputs are the documented data products of the diagnostics data layer:
labeled measurement rows (tdoa_truth.label_measurements), candidate groups
(tdoa_selection.build_candidate_groups), and ReplayRecorder tdoa_records /
state_records. Views (views/*.py) turn these numbers into figures.
"""

import math

from bindings.util.tdoa_truth import interp_ground_truth, is_outlier


def confusion_matrix(labeled_records, threshold):
    """Filter decisions vs ground-truth labels; positive = outlier.

    tp = rejected outlier, fp = rejected good (harmful over-rejection),
    fn = accepted outlier (harmful leak), tn = accepted good.
    Records without a decision (accepted None, degenerate geometry) or
    without a label (no GT coverage) are tallied separately.
    """
    c = {'tp': 0, 'fp': 0, 'fn': 0, 'tn': 0, 'unlabeled': 0, 'undecided': 0}
    for r in labeled_records:
        if r['accepted'] is None:
            c['undecided'] += 1
        elif r['true_error'] is None:
            c['unlabeled'] += 1
        elif r['accepted']:
            c['fn' if is_outlier(r['true_error'], threshold) else 'tn'] += 1
        else:
            c['tp' if is_outlier(r['true_error'], threshold) else 'fp'] += 1
    return c


def selection_regret(labeled_rows):
    """Per candidate group: |true_error| of the selected candidate vs the
    oracle-best candidate. Rows come from
    label_measurements(flatten_candidate_groups(groups), ...).

    Groups whose selected candidate is missing or unlabeled are skipped;
    unlabeled non-selected candidates are ignored within a group.
    """
    by_group = {}
    order = []
    for r in labeled_rows:
        if r['group'] not in by_group:
            by_group[r['group']] = []
            order.append(r['group'])
        by_group[r['group']].append(r)

    out = []
    for gid in order:
        rows = [r for r in by_group[gid] if r['true_error'] is not None]
        selected = [r for r in rows if r['isSelected']]
        if not selected:
            continue
        sel = abs(selected[0]['true_error'])
        best = min(abs(r['true_error']) for r in rows)
        out.append({'group': gid, 't_ms': selected[0]['t_ms'],
                    'selected_abs_error': sel, 'best_abs_error': best,
                    'regret': sel - best})
    return out


def _bin_index(t_ms, t0_ms, bin_ms):
    return int((t_ms - t0_ms) // bin_ms)


def packet_rate_per_anchor(groups, bin_ms=1000.0):
    """{packet anchor id: (bin_centers_ms, packets_per_second)}."""
    if not groups:
        return {}
    t0 = min(g['t_ms'] for g in groups)
    t1 = max(g['t_ms'] for g in groups)
    n_bins = _bin_index(t1, t0, bin_ms) + 1
    counts = {}
    for g in groups:
        c = counts.setdefault(g['idA'], [0] * n_bins)
        c[_bin_index(g['t_ms'], t0, bin_ms)] += 1
    centers = [t0 + (i + 0.5) * bin_ms for i in range(n_bins)]
    return {a: (centers, [n / (bin_ms / 1000.0) for n in c])
            for a, c in counts.items()}


def group_size_histogram(groups):
    hist = {}
    for g in groups:
        n = len(g['candidates'])
        hist[n] = hist.get(n, 0) + 1
    return hist


def find_gaps(t_ms_list, min_gap_ms):
    """[(start_ms, end_ms)] of gaps >= min_gap_ms in a sorted time list."""
    ts = sorted(t_ms_list)
    return [(a, b) for a, b in zip(ts, ts[1:]) if b - a >= min_gap_ms]


def acceptance_rate_series(tdoa_records, bin_ms=1000.0):
    """(bin_centers_ms, accepted/decided fraction per bin; None when a bin
    holds no decided records)."""
    decided = [r for r in tdoa_records if r['accepted'] is not None]
    if not decided:
        return [], []
    t0 = min(r['t_ms'] for r in decided)
    n_bins = _bin_index(max(r['t_ms'] for r in decided), t0, bin_ms) + 1
    n = [0] * n_bins
    n_acc = [0] * n_bins
    for r in decided:
        i = _bin_index(r['t_ms'], t0, bin_ms)
        n[i] += 1
        n_acc[i] += bool(r['accepted'])
    centers = [t0 + (i + 0.5) * bin_ms for i in range(n_bins)]
    return centers, [a / t if t else None for a, t in zip(n_acc, n)]


def nis_series(tdoa_records):
    """Normalized innovation squared per record; finite entries only."""
    out = []
    for r in tdoa_records:
        e, s = r['innovation'], r['innovation_var']
        if math.isfinite(e) and math.isfinite(s) and s > 0.0:
            out.append((r['t_ms'], e * e / s))
    return out


def consistency_series(state_records, gt):
    """Per-iteration estimate-vs-GT error and reported sigma, per axis."""
    rows = []
    for rec in state_records:
        pos = interp_ground_truth(gt, rec['t_ms'])
        if pos is None:
            continue
        rows.append({
            't_ms': rec['t_ms'],
            'err_x': rec['x'] - pos[0], 'sigma_x': math.sqrt(rec['var_x']),
            'err_y': rec['y'] - pos[1], 'sigma_y': math.sqrt(rec['var_y']),
            'err_z': rec['z'] - pos[2], 'sigma_z': math.sqrt(rec['var_z']),
        })
    return rows


def envelope_fraction(rows, k):
    """Fraction of iterations with |err| <= k * sigma, per axis."""
    out = {}
    for axis in ('x', 'y', 'z'):
        if not rows:
            out[axis] = None
            continue
        inside = sum(1 for r in rows
                     if abs(r[f'err_{axis}']) <= k * r[f'sigma_{axis}'])
        out[axis] = inside / len(rows)
    return out


def tdoa_pdop(pairs, anchor_tuples, pos):
    """Position DOP of a set of TDoA pairs at pos; None if unobservable.

    Rows of the geometry matrix are the TDoA measurement jacobian
    d(d_B - d_A)/d(pos) = unit(pos - p_B) - unit(pos - p_A), the same
    expression as measurementModel in mm_tdoa.c.
    """
    import numpy as np

    rows = []
    for id_a, id_b in pairs:
        pa, pb = anchor_tuples.get(id_a), anchor_tuples.get(id_b)
        if pa is None or pb is None:
            continue
        da, db = math.dist(pos, pa), math.dist(pos, pb)
        if da == 0.0 or db == 0.0:
            continue
        rows.append([(pos[k] - pb[k]) / db - (pos[k] - pa[k]) / da
                     for k in range(3)])
    if len(rows) < 3:
        return None
    h = np.array(rows)
    try:
        cov = np.linalg.inv(h.T @ h)
    except np.linalg.LinAlgError:
        return None
    tr = float(np.trace(cov))
    if not math.isfinite(tr) or tr <= 0.0:
        return None
    return math.sqrt(tr)


def dop_series(tdoa_records, gt, anchor_tuples, window_ms=1000.0):
    """[(window_center_ms, pdop | None)] from the pairs accepted per window,
    evaluated at the GT position interpolated at the window center."""
    accepted = [r for r in tdoa_records if r['accepted']]
    if not accepted:
        return []
    t0 = min(r['t_ms'] for r in accepted)
    n_bins = _bin_index(max(r['t_ms'] for r in accepted), t0, window_ms) + 1
    windows = [set() for _ in range(n_bins)]
    for r in accepted:
        windows[_bin_index(r['t_ms'], t0, window_ms)].add((r['idA'], r['idB']))
    out = []
    for i, pairs in enumerate(windows):
        center = t0 + (i + 0.5) * window_ms
        pos = interp_ground_truth(gt, center)
        pdop = tdoa_pdop(sorted(pairs), anchor_tuples, pos) if pos else None
        out.append((center, pdop))
    return out
```

Note the singularity test relies on `np.linalg.inv` raising `LinAlgError` for the rank-deficient x-only geometry; if a platform's LAPACK returns huge values instead, guard by checking `np.linalg.matrix_rank(h) < 3` before inverting — add that check if the test fails.

- [ ] **Step 4: Run the tests**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_analysis.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add tools/tdoa_plot/analysis.py test_python/test_tdoa_analysis.py
git commit -m "feat: numeric analysis core (confusion, regret, NIS, DOP, health)"
```

---

### Task 3: Subcommand scaffolding + `measurement-error` view (Q1)

**Files:**
- Create: `tools/tdoa_plot/views/__init__.py` (empty)
- Create: `tools/tdoa_plot/views/measurement_error.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Produces:
  - `analyze_tdoa._add_run_args(parser, replay=False)` — shared arguments: positional `logfile`, `--anchors` (required), `--save`, `--rebuild-bindings`; with `replay=True` also `--selection-policy` (default `baseline`), `--outlier-filter` (default `integrator`), `--tdoa-model` (default `standard`), `--tdoa-std` (default `0.15`).
  - `views.measurement_error.build_figure(labeled_rows, title) -> Figure` — `labeled_rows` from `label_measurements(flatten_candidate_groups(groups), gt, anchor_tuples)`.
  - Subcommand `measurement-error` wiring the two.
- Later view tasks copy this exact subcommand pattern.

- [ ] **Step 1: Write the failing test**

Create `test_python/test_tdoa_views.py`:

```python
"""Figure smoke tests for the analysis views (need matplotlib, Agg backend)."""

import pytest

matplotlib = pytest.importorskip('matplotlib')
matplotlib.use('Agg')


def _labeled_row(t, idA, idB, dd, true_error, selected=False, group=0):
    return {'t_ms': t, 'idA': idA, 'idB': idB, 'distanceDiff': dd,
            'isSelected': selected, 'group': group,
            'true_diff': dd - true_error if true_error is not None else None,
            'true_error': true_error}


def _synthetic_labeled_rows(n=200):
    rows = []
    for i in range(n):
        idA, idB = i % 4, 4 + i % 3
        err = 0.02 * ((i % 7) - 3) + (1.0 if i % 29 == 0 else 0.0)
        rows.append(_labeled_row(float(i * 50), idA, idB, 0.5 + err, err,
                                 selected=(i % 2 == 0), group=i // 2))
    rows.append(_labeled_row(1e9, 0, 4, 0.0, None))   # unlabeled row
    return rows


def test_measurement_error_figure_renders(tmp_path):
    from tools.tdoa_plot.views.measurement_error import build_figure
    fig = build_figure(_synthetic_labeled_rows(), 'synthetic')
    assert len(fig.axes) == 3
    fig.savefig(tmp_path / 'me.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py -v`
Expected: FAIL with `ModuleNotFoundError` (`tools.tdoa_plot.views`)

- [ ] **Step 3: Implement the view**

Create empty `tools/tdoa_plot/views/__init__.py`, then `tools/tdoa_plot/views/measurement_error.py`:

```python
"""Q1: How good are the raw UWB measurements, per anchor and per pair?

Consumes truth-labeled candidate rows (every candidate, not only selected
ones): GT-referenced error per anchor, error distribution, and the worst
pairs over time. Needs no replay — this view is about the measurements,
not the estimator.
"""

import matplotlib.pyplot as plt

from tools.tdoa_plot.common import CATEGORICAL


def _percentile(sorted_vals, q):
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


def _per_key_errors(rows, key_fn):
    out = {}
    for r in rows:
        if r['true_error'] is None:
            continue
        for k in key_fn(r):
            out.setdefault(k, []).append(r['true_error'])
    return out


def build_figure(labeled_rows, title):
    fig, (ax_anchor, ax_hist, ax_pairs) = plt.subplots(
        3, 1, figsize=(12, 10))

    # Per-anchor aggregate: a bad anchor shows up in all its pairs.
    per_anchor = _per_key_errors(labeled_rows, lambda r: (r['idA'], r['idB']))
    anchors = sorted(per_anchor)
    med, p95 = [], []
    for a in anchors:
        vals = sorted(abs(e) for e in per_anchor[a])
        med.append(_percentile(vals, 0.5))
        p95.append(_percentile(vals, 0.95))
    x = range(len(anchors))
    ax_anchor.bar(x, med, color=CATEGORICAL[0], label='median |error|')
    ax_anchor.plot(x, p95, 'v', color=CATEGORICAL[3], markersize=8,
                   label='p95 |error|')
    ax_anchor.set_xticks(list(x), [str(a) for a in anchors])
    ax_anchor.set_xlabel('anchor id (as either pair member)')
    ax_anchor.set_ylabel('|true error| [m]')
    ax_anchor.legend(loc='upper right')
    ax_anchor.grid(True, alpha=0.3)

    # Error distribution: bias and tails of the whole measurement stream.
    errors = [r['true_error'] for r in labeled_rows
              if r['true_error'] is not None]
    ax_hist.hist(errors, bins=100, color=CATEGORICAL[0])
    ax_hist.axvline(0.0, color='black', lw=1.0, alpha=0.5)
    ax_hist.set_xlabel('true error = measured - GT distanceDiff [m]')
    ax_hist.set_ylabel('measurements')
    ax_hist.grid(True, alpha=0.3)

    # Worst 4 pairs over time (fixed palette assignment, never cycled).
    per_pair = _per_key_errors(labeled_rows, lambda r: ((r['idA'], r['idB']),))
    worst = sorted(per_pair,
                   key=lambda p: -_percentile(sorted(abs(e) for e in per_pair[p]), 0.5))[:4]
    for color, pair in zip(CATEGORICAL, worst):
        pts = [(r['t_ms'] / 1000.0, r['true_error']) for r in labeled_rows
               if (r['idA'], r['idB']) == pair and r['true_error'] is not None]
        ax_pairs.plot([p[0] for p in pts], [p[1] for p in pts], '.',
                      markersize=3, color=color, label=f'pair {pair}')
    ax_pairs.axhline(0.0, color='black', lw=1.0, alpha=0.5)
    ax_pairs.set_xlabel('time [s]')
    ax_pairs.set_ylabel('true error [m]')
    ax_pairs.legend(loc='upper right', title='worst pairs by median |error|')
    ax_pairs.grid(True, alpha=0.3)

    n_unlabeled = sum(1 for r in labeled_rows if r['true_error'] is None)
    fig.suptitle(f'{title} — measurement error vs ground truth '
                 f'({len(errors)} labeled, {n_unlabeled} outside GT coverage)')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

In `tools/tdoa_plot/analyze_tdoa.py` add the shared-args helper and the subcommand (and extend the module docstring's subcommand list — do this for every view task):

```python
def _add_run_args(p, replay=False):
    p.add_argument('logfile', help='uSD log file (binary) recorded on the Crazyflie')
    p.add_argument('--anchors', required=True,
                   help='YAML file mapping anchor id -> {x, y, z}')
    p.add_argument('--save', default=None, metavar='PNG',
                   help='Save the figure to this file instead of showing it')
    p.add_argument('--rebuild-bindings', action='store_true',
                   help='Force a rebuild of the cffirmware bindings')
    if replay:
        p.add_argument('--selection-policy', default='baseline',
                       help='Selection policy for the replay (default: baseline)')
        p.add_argument('--outlier-filter', default='integrator',
                       help='Outlier filter for the replay (default: integrator; '
                            'diagnostics require an externalized filter)')
        p.add_argument('--tdoa-model', choices=('standard', 'robust'),
                       default='standard', help='TDoA measurement model')
        p.add_argument('--tdoa-std', type=float, default=0.15,
                       help='TDoA measurement std dev [m]')


def _load_labeled_candidates(args):
    """Shared Q1/Q2 data prep: truth-labeled rows for every candidate."""
    from bindings.util.tdoa_truth import flatten_candidate_groups, label_measurements
    from tools.tdoa_plot.common import ensure_bindings, load_run
    ensure_bindings(rebuild=args.rebuild_bindings)
    run = load_run(args.logfile, args.anchors)
    if not run['groups']:
        sys.exit('No estTdoaCand data in this log '
                 '(was tdoaEngine.logCand > 0 during logging?)')
    if not run['gt']:
        sys.exit('No Lighthouse ground truth in this log; this view needs '
                 'lighthouse.x/y/z in the fixedFrequency uSD config')
    return run, label_measurements(flatten_candidate_groups(run['groups']),
                                   run['gt'], run['anchor_tuples'])


def cmd_measurement_error(args):
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.measurement_error import build_figure
    run, labeled = _load_labeled_candidates(args)
    fig = build_figure(labeled, Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('measurement-error',
                       help='Q1: GT-referenced raw measurement error per '
                            'anchor/pair (no replay)')
    _add_run_args(p)
    p.set_defaults(func=cmd_measurement_error)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py test_python/test_tdoa_common.py -v && make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py measurement-error ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q1.png`
Expected: figure saved; open it and check labels/legends render without collisions

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/ tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: measurement-error analysis view (Q1)"
```

---

### Task 4: `selection` view (Q2)

**Files:**
- Create: `tools/tdoa_plot/views/selection.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `analysis.selection_regret`, `_load_labeled_candidates` (Task 3).
- Produces: `views.selection.build_figure(regret_rows, title) -> Figure`; subcommand `selection`.

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def test_selection_figure_renders(tmp_path):
    from tools.tdoa_plot.analysis import selection_regret
    from tools.tdoa_plot.views.selection import build_figure
    regret = selection_regret(_synthetic_labeled_rows())
    assert regret, 'synthetic rows must produce regret entries'
    fig = build_figure(regret, 'synthetic')
    assert len(fig.axes) == 2
    fig.savefig(tmp_path / 'sel.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py::test_selection_figure_renders -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/selection.py`**

```python
"""Q2: Does the matcher select good candidates?

Selected-candidate GT error vs the oracle-best candidate of the same packet
("selection regret"), computed without running the estimator. A matcher that
often picks a much worse candidate than the group's best is leaving accuracy
on the table.
"""

import matplotlib.pyplot as plt

from tools.tdoa_plot.common import CATEGORICAL


def build_figure(regret_rows, title):
    fig, (ax_err, ax_cum) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

    t = [r['t_ms'] / 1000.0 for r in regret_rows]
    ax_err.plot(t, [r['selected_abs_error'] for r in regret_rows], '.',
                markersize=3, color=CATEGORICAL[1], label='selected candidate')
    ax_err.plot(t, [r['best_abs_error'] for r in regret_rows], '.',
                markersize=3, color=CATEGORICAL[0], label='oracle-best candidate')
    ax_err.set_ylabel('|true error| [m]')
    ax_err.legend(loc='upper right')
    ax_err.grid(True, alpha=0.3)

    cum = []
    total = 0.0
    for r in regret_rows:
        total += r['regret']
        cum.append(total)
    ax_cum.plot(t, cum, color=CATEGORICAL[3], lw=1.5)
    ax_cum.set_xlabel('time [s]')
    ax_cum.set_ylabel('cumulative regret [m]')
    ax_cum.grid(True, alpha=0.3)

    n = len(regret_rows)
    n_best = sum(1 for r in regret_rows if r['regret'] <= 1e-9)
    mean_regret = total / n if n else 0.0
    fig.suptitle(f'{title} — selection regret: mean {mean_regret:.3f} m, '
                 f'selected == best in {n_best}/{n} groups')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

In `analyze_tdoa.py`:

```python
def cmd_selection(args):
    from tools.tdoa_plot.analysis import selection_regret
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.selection import build_figure
    run, labeled = _load_labeled_candidates(args)
    regret = selection_regret(labeled)
    if not regret:
        sys.exit('No labelable groups with a selected candidate; cannot '
                 'compute selection regret')
    fig = build_figure(regret, Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('selection',
                       help='Q2: selected vs oracle-best candidate GT error, '
                            'selection regret (no replay)')
    _add_run_args(p)
    p.set_defaults(func=cmd_selection)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py selection ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q2.png`
Expected: figure saved; eyeball it

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/selection.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: selection-regret analysis view (Q2)"
```

---

### Task 5: `health` view (Q3)

**Files:**
- Create: `tools/tdoa_plot/views/health.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `analysis.packet_rate_per_anchor`, `analysis.group_size_histogram`, `analysis.find_gaps`.
- Produces: `views.health.build_figure(groups, title) -> Figure`; subcommand `health` (needs no GT and no replay).

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def _synthetic_groups(n=300, anchors=(1, 2, 3)):
    groups = []
    for i in range(n):
        t = float(i * 40)
        if 4000.0 < t < 6000.0:   # a drought
            continue
        groups.append({'group': i, 't_ms': t, 'idA': anchors[i % len(anchors)],
                       'candidates': [{'idB': 10 + j, 'distanceDiff': 0.0,
                                       'isSelected': j == 0}
                                      for j in range(2 + i % 3)]})
    return groups


def test_health_figure_renders(tmp_path):
    from tools.tdoa_plot.views.health import build_figure
    fig = build_figure(_synthetic_groups(), 'synthetic')
    # One rate panel per packet anchor (small multiples) + histogram panel.
    assert len(fig.axes) == 3 + 1
    fig.savefig(tmp_path / 'health.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py::test_health_figure_renders -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/health.py`**

```python
"""Q3: System health — packet rates, candidate-set sizes, droughts.

Per-anchor packet rate as small multiples (one panel per anchor: anchor
counts exceed any safe categorical palette, and rate shapes compare fine
on a shared scale), gaps >= 1 s marked on each panel, and the group-size
histogram. Needs neither GT nor a replay.
"""

import matplotlib.pyplot as plt

from tools.tdoa_plot.analysis import (
    find_gaps, group_size_histogram, packet_rate_per_anchor)
from tools.tdoa_plot.common import CATEGORICAL

GAP_MS = 1000.0


def build_figure(groups, title):
    rates = packet_rate_per_anchor(groups)
    anchors = sorted(rates)
    fig, axes = plt.subplots(len(anchors) + 1, 1, figsize=(12, 2 * len(anchors) + 3),
                             sharex=False)
    gaps = find_gaps([g['t_ms'] for g in groups], GAP_MS)
    max_rate = max((max(hz) for _, hz in rates.values()), default=1.0)

    for ax, anchor in zip(axes, anchors):
        centers, hz = rates[anchor]
        ax.plot([c / 1000.0 for c in centers], hz, color=CATEGORICAL[0], lw=1.5)
        for a, b in gaps:
            ax.axvspan(a / 1000.0, b / 1000.0, color=CATEGORICAL[3], alpha=0.15)
        ax.set_ylim(0.0, max_rate * 1.1)
        ax.set_ylabel(f'anchor {anchor}\n[packets/s]')
        ax.grid(True, alpha=0.3)
    axes[len(anchors) - 1].set_xlabel('time [s]')

    hist = group_size_histogram(groups)
    sizes = sorted(hist)
    axes[-1].bar(sizes, [hist[s] for s in sizes], color=CATEGORICAL[0])
    axes[-1].set_xlabel('candidates per packet')
    axes[-1].set_ylabel('packets')
    axes[-1].set_xticks(sizes)
    axes[-1].grid(True, alpha=0.3)

    gap_note = (f', {len(gaps)} gaps >= {GAP_MS / 1000.0:.0f}s (shaded)'
                if gaps else '')
    fig.suptitle(f'{title} — packet health: {len(groups)} packets, '
                 f'{len(anchors)} transmitting anchors{gap_note}')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

```python
def cmd_health(args):
    from tools.tdoa_plot.common import ensure_bindings, load_run, save_or_show
    from tools.tdoa_plot.views.health import build_figure
    ensure_bindings(rebuild=args.rebuild_bindings)
    run = load_run(args.logfile, args.anchors)
    if not run['groups']:
        sys.exit('No estTdoaCand data in this log '
                 '(was tdoaEngine.logCand > 0 during logging?)')
    fig = build_figure(run['groups'], Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('health',
                       help='Q3: per-anchor packet rate, group sizes, '
                            'droughts (no replay, no GT needed)')
    _add_run_args(p)
    p.set_defaults(func=cmd_health)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py health ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q3.png`
Expected: figure saved; eyeball it

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/health.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: packet-health analysis view (Q3)"
```

---

### Task 6: `filter-decisions` view (Q4)

**Files:**
- Create: `tools/tdoa_plot/views/filter_decisions.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `common.recorded_replay`, `tdoa_truth.label_measurements` (on `tdoa_records`), `analysis.confusion_matrix`.
- Produces: `views.filter_decisions.build_figure(labeled_records, confusion, threshold, filter_name, title) -> Figure`; subcommand `filter-decisions` with `--outlier-threshold` (default `0.5` m).

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def _synthetic_tdoa_records(n=300, filter_state='integrator'):
    records = []
    for i in range(n):
        outlier = i % 23 == 0
        err = (1.2 if outlier else 0.05 * ((i % 5) - 2))
        state = ({'integrator': min(300.0, 30.0 * i), 'is_open': i < 100}
                 if filter_state == 'integrator'
                 else {'n': min(i, 50), 'median': 0.0, 'mad': 0.1})
        records.append({
            't_ms': float(i * 40), 'idA': i % 4, 'idB': 4 + i % 3,
            'distanceDiff': 0.5, 'stdDev': 0.15,
            'innovation': err, 'innovation_var': 0.05,
            'accepted': not outlier or i % 46 == 0,   # some outliers leak
            'filter_state': state,
            'true_diff': 0.5 - err, 'true_error': err,
        })
    records.append({**records[-1], 't_ms': 1e9, 'accepted': None,
                    'innovation': float('nan'), 'true_error': None})
    return records


def test_filter_decisions_figure_renders(tmp_path):
    from tools.tdoa_plot.analysis import confusion_matrix
    from tools.tdoa_plot.views.filter_decisions import build_figure
    records = _synthetic_tdoa_records()
    fig = build_figure(records, confusion_matrix(records, 0.5), 0.5,
                       'integrator', 'synthetic')
    assert len(fig.axes) == 3
    fig.savefig(tmp_path / 'fd.png')


def test_filter_decisions_figure_without_integrator_state(tmp_path):
    # A filter with a different snapshot schema still renders (panel adapts).
    from tools.tdoa_plot.analysis import confusion_matrix
    from tools.tdoa_plot.views.filter_decisions import build_figure
    records = _synthetic_tdoa_records(filter_state='mad')
    fig = build_figure(records, confusion_matrix(records, 0.5), 0.5,
                       'mad_window', 'synthetic')
    fig.savefig(tmp_path / 'fd2.png')
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py -k filter_decisions -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/filter_decisions.py`**

```python
"""Q4: What did the outlier filter do, and was it right?

Innovations over time split by decision (accepted/rejected — color AND
marker shape), the filter's internal state on its own panel (never a twin
axis), and the GT-labeled confusion summary: rejected-good (fp) is wasted
signal, accepted-bad (fn) is what fly-aways are made of.
"""

import math

import matplotlib.pyplot as plt

from tools.tdoa_plot.common import COLOR_ACCEPTED, COLOR_REJECTED


def _decision_panel(ax, records, threshold):
    for accepted, color, marker, label in [
            (True, COLOR_ACCEPTED, '.', 'accepted'),
            (False, COLOR_REJECTED, 'x', 'rejected')]:
        pts = [(r['t_ms'] / 1000.0, r['innovation']) for r in records
               if r['accepted'] is accepted and math.isfinite(r['innovation'])]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], marker,
                markersize=4, linestyle='none', color=color,
                label=f'{label} ({len(pts)})')
    ax.axhline(threshold, color='black', lw=1.0, alpha=0.5)
    ax.axhline(-threshold, color='black', lw=1.0, alpha=0.5,
               label=f'GT outlier threshold ±{threshold} m')
    ax.set_ylabel('innovation [m]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _state_panel(ax, records):
    t = [r['t_ms'] / 1000.0 for r in records]
    states = [r['filter_state'] for r in records]
    if any('integrator' in s for s in states):
        ax.plot(t, [s.get('integrator') for s in states],
                color=COLOR_ACCEPTED, lw=1.0, label='integrator level')
        closed = [i for i, s in enumerate(states) if s.get('is_open') is False]
        if closed:
            ax.plot([t[i] for i in closed],
                    [states[i].get('integrator') for i in closed], '.',
                    markersize=2, color=COLOR_REJECTED, label='gate closed')
        ax.set_ylabel('integrator [ms]')
    elif any('median' in s for s in states):
        ax.plot(t, [s.get('median') for s in states],
                color=COLOR_ACCEPTED, lw=1.0, label='window median [m]')
        ax.plot(t, [s.get('mad') for s in states],
                color=COLOR_REJECTED, lw=1.0, label='window MAD [m]')
        ax.set_ylabel('[m]')
    else:
        ax.text(0.5, 0.5, 'filter exposes no snapshot state',
                ha='center', va='center', transform=ax.transAxes)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _confusion_panel(ax, c):
    ax.axis('off')
    decided = c['tp'] + c['fp'] + c['fn'] + c['tn']
    lines = [
        f"decided & labeled: {decided}   "
        f"unlabeled: {c['unlabeled']}   undecided (NaN): {c['undecided']}",
        f"rejected outliers (tp): {c['tp']}    "
        f"accepted good (tn): {c['tn']}",
        f"REJECTED GOOD (fp, wasted signal): {c['fp']}"
        + (f"  = {c['fp'] / max(1, c['fp'] + c['tn']):.1%} of good" if decided else ''),
        f"ACCEPTED OUTLIERS (fn, leaked): {c['fn']}"
        + (f"  = {c['fn'] / max(1, c['fn'] + c['tp']):.1%} of outliers" if decided else ''),
    ]
    ax.text(0.02, 0.9, '\n'.join(lines), va='top', family='monospace',
            transform=ax.transAxes)


def build_figure(labeled_records, confusion, threshold, filter_name, title):
    fig, (ax_dec, ax_state, ax_conf) = plt.subplots(
        3, 1, figsize=(12, 10), sharex=False,
        gridspec_kw={'height_ratios': [3, 2, 1]})
    ax_state.sharex(ax_dec)

    _decision_panel(ax_dec, labeled_records, threshold)
    _state_panel(ax_state, labeled_records)
    ax_state.set_xlabel('time [s]')
    _confusion_panel(ax_conf, confusion)

    fig.suptitle(f"{title} — outlier filter '{filter_name}' decisions vs "
                 f"ground truth (threshold {threshold} m)")
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

In `analyze_tdoa.py`:

```python
def _load_recorded(args):
    """Shared Q4-Q7 data prep: recorded replay + truth-labeled records."""
    from bindings.util.tdoa_truth import label_measurements
    from tools.tdoa_plot.common import ensure_bindings, load_run, recorded_replay
    ensure_bindings(rebuild=args.rebuild_bindings)
    run = load_run(args.logfile, args.anchors)
    if not run['groups']:
        sys.exit('No estTdoaCand data in this log '
                 '(was tdoaEngine.logCand > 0 during logging?)')
    print(f"Replaying (policy: {args.selection_policy}, "
          f"filter: {args.outlier_filter}, model: {args.tdoa_model}) ...")
    trajectory, recorder = recorded_replay(
        run, selection_policy=args.selection_policy,
        outlier_filter=args.outlier_filter, tdoa_model=args.tdoa_model,
        tdoa_std=args.tdoa_std)
    labeled = label_measurements(recorder.tdoa_records, run['gt'],
                                 run['anchor_tuples'])
    if recorder.n_skipped_unknown_anchor:
        print(f'WARNING: {recorder.n_skipped_unknown_anchor} samples skipped '
              f'(anchor not in the anchors file)')
    return run, trajectory, recorder, labeled


def cmd_filter_decisions(args):
    from tools.tdoa_plot.analysis import confusion_matrix
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.filter_decisions import build_figure
    run, _, recorder, labeled = _load_recorded(args)
    if not run['gt']:
        print('NOTE: no ground truth — confusion counts will be all-unlabeled')
    fig = build_figure(labeled,
                       confusion_matrix(labeled, args.outlier_threshold),
                       args.outlier_threshold, args.outlier_filter,
                       Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('filter-decisions',
                       help='Q4: innovations, accept/reject decisions, filter '
                            'state and GT confusion matrix')
    _add_run_args(p, replay=True)
    p.add_argument('--outlier-threshold', type=float, default=0.5,
                   help='|true error| [m] above which a measurement counts '
                        'as a GT outlier (default: 0.5)')
    p.set_defaults(func=cmd_filter_decisions)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py filter-decisions ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q4.png`
Expected: figure saved; check the confusion numbers are plausible (few fn/fp on the known-good run)

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/filter_decisions.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: filter-decisions analysis view (Q4)"
```

---

### Task 7: `consistency` view (Q5)

**Files:**
- Create: `tools/tdoa_plot/views/consistency.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `analysis.consistency_series`, `analysis.envelope_fraction`, `analysis.nis_series`, `_load_recorded` (Task 6).
- Produces: `views.consistency.build_figure(rows, fractions, nis, k, title) -> Figure`; subcommand `consistency` with `--sigma-k` (default `3.0`).

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def _synthetic_state_records(n=500):
    return [{'t_ms': float(i * 10),
             'x': 0.05 * ((i % 9) - 4), 'y': 0.0, 'z': 1.0 + 0.02 * (i % 3),
             'var_x': 0.01, 'var_y': 0.01, 'var_z': 0.02}
            for i in range(n)]


def test_consistency_figure_renders(tmp_path):
    from tools.tdoa_plot.analysis import (
        consistency_series, envelope_fraction, nis_series)
    from tools.tdoa_plot.views.consistency import build_figure
    gt = [(0.0, (0.0, 0.0, 1.0)), (5000.0, (0.0, 0.0, 1.0))]
    rows = consistency_series(_synthetic_state_records(), gt)
    nis = nis_series(_synthetic_tdoa_records())
    fig = build_figure(rows, envelope_fraction(rows, 3.0), nis, 3.0, 'synthetic')
    assert len(fig.axes) == 4   # x, y, z envelopes + NIS
    fig.savefig(tmp_path / 'cons.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py::test_consistency_figure_renders -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/consistency.py`**

```python
"""Q5: Is the Kalman filter honest about its uncertainty?

Per-axis estimate-vs-GT error inside the filter's own ±k*sigma envelope
(overconfident: error routinely escapes the band; underconfident: band
dwarfs the error), plus per-measurement NIS against its chi-square(1)
expectation (95% of NIS values should sit below 3.84 for a consistent,
well-tuned tdoa_std).
"""

import matplotlib.pyplot as plt

from tools.tdoa_plot.common import CATEGORICAL

CHI2_1_P95 = 3.841


def build_figure(rows, fractions, nis, k, title):
    fig, axes = plt.subplots(4, 1, figsize=(12, 11))
    t = [r['t_ms'] / 1000.0 for r in rows]

    for ax, axis in zip(axes[:3], ('x', 'y', 'z')):
        err = [r[f'err_{axis}'] for r in rows]
        sig = [r[f'sigma_{axis}'] for r in rows]
        ax.fill_between(t, [-k * s for s in sig], [k * s for s in sig],
                        color=CATEGORICAL[0], alpha=0.15,
                        label=f'±{k:g}σ (filter)')
        ax.plot(t, err, color=CATEGORICAL[3], lw=0.8, label='error vs GT')
        frac = fractions[axis]
        ax.set_ylabel(f'{axis} [m]')
        ax.set_title(f'{axis}: {frac:.1%} of samples inside ±{k:g}σ'
                     if frac is not None else f'{axis}: no GT overlap',
                     fontsize=10, loc='left')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    ax_nis = axes[3]
    if nis:
        ax_nis.plot([p[0] / 1000.0 for p in nis], [p[1] for p in nis], '.',
                    markersize=2, color=CATEGORICAL[0], label='NIS')
        below = sum(1 for _, v in nis if v <= CHI2_1_P95) / len(nis)
        ax_nis.axhline(CHI2_1_P95, color='black', lw=1.0, alpha=0.5,
                       label=f'χ²(1) p95 = {CHI2_1_P95} '
                             f'({below:.1%} below; expect ~95%)')
        ax_nis.set_yscale('log')
    ax_nis.set_xlabel('time [s]')
    ax_nis.set_ylabel('NIS = e²/S')
    ax_nis.legend(loc='upper right')
    ax_nis.grid(True, alpha=0.3)

    fig.suptitle(f'{title} — filter consistency (envelope + NIS)')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

```python
def cmd_consistency(args):
    from tools.tdoa_plot.analysis import (
        consistency_series, envelope_fraction, nis_series)
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.consistency import build_figure
    run, _, recorder, labeled = _load_recorded(args)
    if not run['gt']:
        sys.exit('No Lighthouse ground truth in this log; the consistency '
                 'view needs lighthouse.x/y/z in the fixedFrequency config')
    rows = consistency_series(recorder.state_records, run['gt'])
    fig = build_figure(rows, envelope_fraction(rows, args.sigma_k),
                       nis_series(recorder.tdoa_records), args.sigma_k,
                       Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('consistency',
                       help='Q5: error vs ±kσ envelope per axis, NIS vs χ²')
    _add_run_args(p, replay=True)
    p.add_argument('--sigma-k', type=float, default=3.0,
                   help='Envelope width in sigmas (default: 3)')
    p.set_defaults(func=cmd_consistency)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py consistency ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q5.png`
Expected: figure saved; check the envelope fractions and NIS-below-p95 numbers print on the figure

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/consistency.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: filter-consistency analysis view (Q5)"
```

---

### Task 8: `geometry` view (Q7)

**Files:**
- Create: `tools/tdoa_plot/views/geometry.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `analysis.dop_series`, `_load_recorded` (Task 6).
- Produces: `views.geometry.build_figure(dop_rows, title) -> Figure`; subcommand `geometry` with `--window` (seconds, default `1.0`).

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def test_geometry_figure_renders(tmp_path):
    from tools.tdoa_plot.views.geometry import build_figure
    dop_rows = [(500.0, 1.8), (1500.0, 2.4), (2500.0, None), (3500.0, 9.0)]
    fig = build_figure(dop_rows, 'synthetic')
    assert len(fig.axes) == 1
    fig.savefig(tmp_path / 'geom.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py::test_geometry_figure_renders -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/geometry.py`**

```python
"""Q7: Was the constellation geometry ever the problem?

Position DOP of the anchor pairs actually accepted per time window,
evaluated at the GT position: a high-DOP window explains a bad estimate
even with perfect measurements; a low-DOP window with a bad estimate points
at the measurements or the filter instead. Windows with unobservable
geometry (< 3 usable pairs or rank-deficient) are marked, not hidden.
"""

import matplotlib.pyplot as plt

from tools.tdoa_plot.common import CATEGORICAL


def build_figure(dop_rows, title):
    fig, ax = plt.subplots(figsize=(12, 4))
    ok = [(t / 1000.0, v) for t, v in dop_rows if v is not None]
    bad = [t / 1000.0 for t, v in dop_rows if v is None]
    if ok:
        ax.plot([p[0] for p in ok], [p[1] for p in ok], '-',
                color=CATEGORICAL[0], lw=1.5, label='TDoA PDOP at GT position')
    for i, t in enumerate(bad):
        ax.axvline(t, color=CATEGORICAL[3], alpha=0.4, lw=1.0,
                   label='unobservable window' if i == 0 else None)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('PDOP')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    n_bad = len(bad)
    fig.suptitle(f'{title} — accepted-pair geometry '
                 f'({len(dop_rows)} windows, {n_bad} unobservable)')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

```python
def cmd_geometry(args):
    from tools.tdoa_plot.analysis import dop_series
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.geometry import build_figure
    run, _, recorder, _ = _load_recorded(args)
    if not run['gt']:
        sys.exit('No Lighthouse ground truth in this log; the geometry view '
                 'evaluates DOP at the GT position')
    dop_rows = dop_series(recorder.tdoa_records, run['gt'],
                          run['anchor_tuples'],
                          window_ms=args.window * 1000.0)
    fig = build_figure(dop_rows, Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('geometry',
                       help='Q7: TDoA PDOP of the accepted pairs at the GT '
                            'position, per time window')
    _add_run_args(p, replay=True)
    p.add_argument('--window', type=float, default=1.0,
                   help='DOP window [s] (default: 1.0)')
    p.set_defaults(func=cmd_geometry)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py geometry ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q7.png`
Expected: figure saved; PDOP magnitudes plausible (roughly 1–10 for a sane constellation)

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/geometry.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: geometry (DOP) analysis view (Q7)"
```

---

### Task 9: `forensic` view (Q6)

**Files:**
- Create: `tools/tdoa_plot/views/forensic.py`
- Modify: `tools/tdoa_plot/analyze_tdoa.py`
- Test: `test_python/test_tdoa_views.py`

**Interfaces:**
- Consumes: `analysis.acceptance_rate_series`, `analysis.consistency_series`, `analysis.packet_rate_per_anchor`, `interp_ground_truth`; `_load_recorded`.
- Produces: `views.forensic.build_figure(trajectory, gt, labeled_records, state_records, groups, title) -> Figure` — the shared-time-axis composite; subcommand `forensic`.

- [ ] **Step 1: Write the failing test**

Add to `test_python/test_tdoa_views.py`:

```python
def test_forensic_figure_renders(tmp_path):
    from tools.tdoa_plot.views.forensic import build_figure
    gt = [(0.0, (0.0, 0.0, 1.0)), (12000.0, (0.0, 0.0, 1.0))]
    trajectory = [(float(t), (0.02, 0.0, 1.0)) for t in range(0, 12000, 10)]
    fig = build_figure(trajectory, gt, _synthetic_tdoa_records(),
                       _synthetic_state_records(), _synthetic_groups(),
                       'synthetic')
    assert len(fig.axes) == 4
    # All time panels share one x-axis (forensic zooming works).
    assert all(ax.get_shared_x_axes().joined(ax, fig.axes[0])
               for ax in fig.axes[1:])
    fig.savefig(tmp_path / 'forensic.png')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_views.py::test_forensic_figure_renders -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Implement `tools/tdoa_plot/views/forensic.py`**

```python
"""Q6: Why is the estimate bad HERE?

One shared time axis stacking the position error against everything that
could explain it: the filter's acceptance rate, the innovation magnitudes,
the total packet rate (droughts), and the filter's own sigma. Open it
interactively and zoom into the bad moment; each panel names a different
suspect (measurements / filter / geometry / drought).
"""

import math

import matplotlib.pyplot as plt

from bindings.util.tdoa_truth import interp_ground_truth
from tools.tdoa_plot.analysis import acceptance_rate_series
from tools.tdoa_plot.common import CATEGORICAL, COLOR_ACCEPTED, COLOR_REJECTED


def build_figure(trajectory, gt, labeled_records, state_records, groups, title):
    fig, (ax_err, ax_acc, ax_innov, ax_rate) = plt.subplots(
        4, 1, sharex=True, figsize=(12, 11))

    # Panel 1: 3D position error vs GT, with the filter's 3D sigma alongside
    # (same scale, same unit — one axis).
    err_pts = []
    for t_ms, pos in trajectory:
        gt_pos = interp_ground_truth(gt, t_ms)
        if gt_pos is None or not all(math.isfinite(c) for c in pos):
            continue
        err_pts.append((t_ms / 1000.0,
                        math.dist(pos, gt_pos)))
    ax_err.plot([p[0] for p in err_pts], [p[1] for p in err_pts],
                color=CATEGORICAL[3], lw=1.0, label='3D error vs GT')
    sig = [(r['t_ms'] / 1000.0,
            math.sqrt(r['var_x'] + r['var_y'] + r['var_z']))
           for r in state_records]
    ax_err.plot([p[0] for p in sig], [p[1] for p in sig],
                color=CATEGORICAL[0], lw=1.0, alpha=0.7,
                label='filter 3D sigma')
    ax_err.set_ylabel('[m]')
    ax_err.legend(loc='upper right')
    ax_err.grid(True, alpha=0.3)

    # Panel 2: acceptance rate (the filter's behavior).
    centers, fractions = acceptance_rate_series(labeled_records)
    pts = [(c / 1000.0, f) for c, f in zip(centers, fractions) if f is not None]
    ax_acc.plot([p[0] for p in pts], [p[1] for p in pts],
                color=COLOR_ACCEPTED, lw=1.5)
    ax_acc.set_ylim(-0.05, 1.05)
    ax_acc.set_ylabel('accepted\nfraction')
    ax_acc.grid(True, alpha=0.3)

    # Panel 3: innovation magnitude by decision (the measurements' behavior).
    for accepted, color, marker, label in [
            (True, COLOR_ACCEPTED, '.', 'accepted'),
            (False, COLOR_REJECTED, 'x', 'rejected')]:
        pts = [(r['t_ms'] / 1000.0, abs(r['innovation']))
               for r in labeled_records
               if r['accepted'] is accepted and math.isfinite(r['innovation'])]
        ax_innov.plot([p[0] for p in pts], [p[1] for p in pts], marker,
                      markersize=3, linestyle='none', color=color, label=label)
    ax_innov.set_yscale('log')
    ax_innov.set_ylabel('|innovation|\n[m]')
    ax_innov.legend(loc='upper right')
    ax_innov.grid(True, alpha=0.3)

    # Panel 4: total packet rate (droughts starve the estimator).
    t_groups = sorted(g['t_ms'] for g in groups)
    if t_groups:
        bin_ms = 1000.0
        t0 = t_groups[0]
        n_bins = int((t_groups[-1] - t0) // bin_ms) + 1
        counts = [0] * n_bins
        for t in t_groups:
            counts[int((t - t0) // bin_ms)] += 1
        ax_rate.plot([(t0 + (i + 0.5) * bin_ms) / 1000.0 for i in range(n_bins)],
                     counts, color=CATEGORICAL[2], lw=1.5)
    ax_rate.set_ylabel('packets/s')
    ax_rate.set_xlabel('time [s]')
    ax_rate.grid(True, alpha=0.3)

    fig.suptitle(f'{title} — forensic timeline (zoom into the bad moment)')
    fig.tight_layout()
    return fig
```

- [ ] **Step 4: Wire the subcommand**

```python
def cmd_forensic(args):
    from tools.tdoa_plot.common import save_or_show
    from tools.tdoa_plot.views.forensic import build_figure
    run, trajectory, recorder, labeled = _load_recorded(args)
    if not run['gt']:
        sys.exit('No Lighthouse ground truth in this log; the forensic view '
                 'is anchored on the GT position error')
    fig = build_figure(trajectory, run['gt'], labeled,
                       recorder.state_records, run['groups'],
                       Path(args.logfile).name)
    save_or_show(fig, args.save)
```

and in `main()`:

```python
    p = sub.add_parser('forensic',
                       help='Q6: composite timeline — position error vs '
                            'acceptance, innovations, packet rate, sigma')
    _add_run_args(p, replay=True)
    p.set_defaults(func=cmd_forensic)
```

- [ ] **Step 5: Run the tests and the subcommand**

Run: `make test_python`
Expected: all PASS
Run: `cd tools/tdoa_plot && uv run analyze_tdoa.py forensic ../../run_validation.bin --anchors ../../anchors.yaml --save /tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/q6.png`
Expected: figure saved; the four panels align on one time axis

- [ ] **Step 6: Commit**

```bash
git add tools/tdoa_plot/views/forensic.py tools/tdoa_plot/analyze_tdoa.py \
    test_python/test_tdoa_views.py
git commit -m "feat: forensic composite timeline view (Q6)"
```

---

### Task 10: Docs + full validation on the real run

**Files:**
- Modify: `tools/tdoa_plot/README.md`
- Modify: `tools/usdlog/README_tdoa_candidates.md` (analysis section points at the new subcommands)

- [ ] **Step 1: Full suite green**

Run: `make test_python`
Expected: all PASS

- [ ] **Step 2: Render every view on the real run**

```bash
cd tools/tdoa_plot
for v in "measurement-error" "selection" "health" "filter-decisions" "consistency" "geometry" "forensic"; do
  uv run analyze_tdoa.py "$v" ../../run_validation.bin --anchors ../../anchors.yaml \
      --save "/tmp/claude-1000/-home-rik-dev-crazyflie-firmware/294a05d3-7bca-4802-85c0-df4b863b2e25/scratchpad/final_${v}.png" || echo "FAILED: $v"
done
uv run analyze_tdoa.py compare ../../run_validation.bin --anchors ../../anchors.yaml \
    --policies baseline all median round_robin --outlier-filters integrator mad_window pair_integrator
```

Expected: seven PNGs, no `FAILED:` lines, and the compare table prints. Open each PNG (step 7 of the dataviz procedure): check for label collisions, unreadable legends, empty panels. Sanity-check headline numbers against the known-good run: measurement-error histogram roughly centered on 0; selection regret small for `baseline`; confusion matrix mostly tn with few fn; consistency envelope fractions high (≈90%+); PDOP in a plausible 1–10 band; forensic panels aligned.

- [ ] **Step 3: Update the READMEs**

- `tools/tdoa_plot/README.md`: document `analyze_tdoa.py` — one line per subcommand (the Q1–Q8 table from the spec, condensed), the shared flags, and the statement that `plot_tdoa.py` remains the plain trajectory view.
- `tools/usdlog/README_tdoa_candidates.md`: in the analysis section, point to `analyze_tdoa.py <subcommand>` as the analysis entry point.

- [ ] **Step 4: Commit**

```bash
git add tools/tdoa_plot/README.md tools/usdlog/README_tdoa_candidates.md
git commit -m "docs: analyze_tdoa view suite documentation"
```

---

## Execution notes

- Task order: 1 → 2 → 3 → … → 10. Tasks 4–9 all depend on 1–3 (glue, analysis core, subcommand scaffolding); among themselves 4–9 are independent and may be reordered.
- The interactive (no `--save`) paths and figure aesthetics are expected to be iterated on with the user after this plan lands — the figure builders are deliberately thin over the tested numeric core.
