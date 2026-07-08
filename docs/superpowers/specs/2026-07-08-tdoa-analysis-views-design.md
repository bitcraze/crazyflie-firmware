# TDoA diagnostics data layer + analysis views — design

- **Status:** approved by user 2026-07-08
- **Branch:** `tdoa-candidate-logging`
- **Author:** Rik Bouwmeester (with Claude Code)
- **Date:** 2026-07-08
- **Builds on:** `2026-07-02-tdoa-candidate-replay-foundation-design.md`,
  `2026-07-07-tdoa-measurement-model-switch-design.md`,
  `2026-07-07-tdoa-outlier-filters-design.md`

---

## Context

The capture-and-replay foundation works: candidate logs replay through the real
firmware Kalman core with pluggable selection policies and outlier filters, and
`plot_tdoa.py` compares replayed vs live vs Lighthouse ground truth. But
position traces alone do not explain *why* an estimate is good or bad, so the
tooling cannot yet inform tdoa3 improvements.

Two constraints shape this design:

1. **Upstreamability.** The bindings and their utilities are intended for the
   `master` branch. Exposure (binding new firmware internals, replay
   instrumentation, ground-truth math) is mergeable; analysis (plots, scoring,
   experimental algorithms) is not, and may be stripped entirely.
2. **Ground truth is a superpower.** The logs carry Lighthouse ground truth and
   anchor positions, so every logged candidate has a computable *true*
   distanceDiff. Each of the thousands of measurements gets a ground-truth
   error label, independent of the Kalman filter. Most views build on this.

## Goal

1. Make the repository layout enforce the exposure/analysis split (**the cut
   line**), moving existing research payload above it.
2. Build a **diagnostics data layer** (below the line) that serves all
   identified analysis questions.
3. Build the **analysis views** (above the line) that answer those questions.

If work stops after any phase, the tree is consistent and the remaining views
are specified here in enough detail to build later.

## The cut line

**Below the line** — upstream quality, plot-free, unit-tested:
firmware analysis helpers (`src/`), SWIG interface (`bindings/cffirmware.i`),
and `bindings/util/`. Contents are *exposure*: mechanisms (plug-points,
replay seam, recorder), firmware-parity implementations, firmware-mirroring
math, and ground-truth geometry. Reference examples of a plug-point are
allowed when trivial and documented as such.

**Above the line** — disposable analysis: `tools/tdoa_plot/` only. Plots,
scoring, experimental selection policies and outlier filters, and the
numeric analysis they need (regret, confusion matrices, DOP).

Documentation requirements below the line:

- Firmware-parity classes (`BaselinePolicy`, `IntegratorFilter`) must state in
  their docstring that they mirror live firmware behavior (largely present
  today; make it uniform).
- Reference examples (`AllCandidatesPolicy`, `NoneFilter`) must state in their
  docstring that they are kept as trivial reference examples of the plug-point
  contract (0..many measurements per group; accept-everything control).

## Analysis questions (the requirements)

Every view answers one question and consumes only documented data products.

| # | Question | View (subcommand) | Consumes |
|---|---|---|---|
| Q1 | How good are the raw UWB measurements per anchor / per pair? Bias, spread, multipath vs position? | `measurement-error` | candidate groups + truth |
| Q2 | Does the matcher select good candidates? Regret vs oracle-best; selection-policy A/B without running the estimator | `selection` | candidate groups + truth |
| Q3 | System health: packet/candidate rates per anchor, group sizes, measurement droughts | `health` | candidate groups |
| Q4 | What did the outlier filter do, and was it right? Innovation series, accept/reject decisions, GT-labeled confusion matrix, filter internal state | `filter-decisions` | tdoa_records + truth |
| Q5 | Is the Kalman filter honest about its uncertainty? Error vs ±kσ per axis; NIS vs χ² expectation | `consistency` | state_records + tdoa_records + GT |
| Q6 | Why is the estimate bad *here*? Composite shared-time-axis forensic timeline | `forensic` | all of the above |
| Q7 | Was the constellation geometry ever the problem? TDoA DOP at GT position from accepted pairs | `geometry` | tdoa_records + GT + anchors |
| Q8 | Which policy × filter × model combination scores best? (existing `replay_tdoa` table, relocated) | `compare` | trajectories + GT |

## Architecture

Three data products cross the cut line; everything above consumes only these:

1. **Candidate groups** — `tdoa_selection.build_candidate_groups` (exists).
2. **Replay diagnostics** — a `ReplayRecorder` filled during `replay()`:
   `tdoa_records` (per TDoA sample) and `state_records` (per 1 kHz iteration).
   Plain dicts/lists, serializable, so analysis could live out-of-tree.
3. **Truth** — `tdoa_truth.py`: GT extraction/interpolation and true
   distanceDiff / error labeling.

---

## Phase 0 — the split (restructure + seam plumbing, no new analysis capability)

Move existing research payload above the line. Each move is a relocation;
behavior of every CLI is unchanged. The one API addition is the
instance-accepting seam change below, which the split requires.

**Moves out of `bindings/util/tdoa_selection.py`:** `MedianPolicy`,
`RoundRobinPolicy`. Keep `BaselinePolicy` (parity) and `AllCandidatesPolicy`
(reference example). Core `POLICY_FACTORIES` shrinks to `baseline`, `all`.

**Moves out of `bindings/util/tdoa_outlier.py`:** `SanityFilter`,
`MadWindowFilter`, `PairIntegratorFilter`. Keep `IntegratorFilter` (parity)
and `NoneFilter` (reference control). `_is_physically_possible` /
`_anchor_distance_sq` stay below the line (they mirror `outlierFilterTdoa.c`)
and lose the leading underscore — they become public API of the module, since
the moved filters import them from above the line. Core `FILTER_FACTORIES`
shrinks to `integrator`, `none`.

**Destination:** new `tools/tdoa_plot/experiments.py` holding the five moved
classes plus `EXPERIMENTAL_POLICY_FACTORIES`, `EXPERIMENTAL_FILTER_FACTORIES`
and resolvers `resolve_policy(name)` / `resolve_outlier_filter(name)` that
merge the experimental registries over the core ones. CLIs use the resolvers,
so all names keep working.

**Seam change (instances, not just names):**
`replay(params={'outlier_filter': ...})` currently resolves filters by name
inside `bindings/util`, which cannot know experimental names. `outlier_filter`
accepts an `OutlierFilter` *instance* in addition to a core name string.
(`apply_policy` already takes policy instances — no change.)

**Dissolve `tools/usdlog/replay_tdoa.py`:**

- Exposure parts move down: `extract_ground_truth`, `_interp_ground_truth`
  (public: `interp_ground_truth`) into new `bindings/util/tdoa_truth.py`.
- Analysis parts move up: `score_trajectory`, `write_csv`, and the
  policy × filter comparison CLI become the `compare` subcommand of a new
  `tools/tdoa_plot/analyze_tdoa.py` (argparse subparsers; further subcommands
  arrive in phase 3). The log summary + `verify_baseline_reconstruction`
  report printed by `replay_tdoa.py` moves with it.
- `tools/usdlog/` keeps plumbing only: `cfusdlog.py`, `read_usd_log.sh`,
  `anchors_from_cfcli.py`, capture configs, `plot_events.py`, `example.py`.
- Update `README_tdoa_candidates.md`, `plot_tdoa.py` imports
  (`extract_ground_truth` etc. from `tdoa_truth`), and docs referencing
  `replay_tdoa`.

**Tests:** parity/seam/mechanism tests keep importing `bindings.util`.
Tests for moved classes move to a new `test_python/test_tdoa_experiments.py`
importing `tools.tdoa_plot.experiments` (precedent: `test_tdoa_plot.py`).
`test_tdoa_replay.py`'s imports of `replay_tdoa` helpers retarget to
`tdoa_truth` / `analyze_tdoa`.

**Docstrings:** apply the documentation requirements from "The cut line".

## Phase 1 — binding additions + replay instrumentation

### C / SWIG surface (the only firmware-side changes)

- **`bindings/cffirmware.i`** (`%inline`, pattern of `poly4d_get`):
  `float kalmanCoreGetCovarianceElement(const kalmanCoreData_t*, int i, int j)`
  — returns `P[i][j]`; NaN when `i`/`j` outside `[0, KC_STATE_DIM)`. Composes
  with the already-exposed `KC_STATE_X/Y/Z` (verified: 0/1/2, DIM 9).
- **`mm_tdoa.c` / `mm_tdoa.h`:**
  `float kalmanCoreTdoaInnovationVariance(const kalmanCoreData_t*, const tdoaMeasurement_t*)`
  — companion of the existing non-flight helper `kalmanCoreTdoaInnovation`;
  runs `measurementModel` and returns S = HPHᵀ + R (H the 1×DIM measurement
  row vector, R = stdDev²), NaN on
  degenerate geometry (same condition as the innovation helper). Plain loop
  over `KC_STATE_DIM`; not used by flight code, documented as such.

### `ReplayRecorder` (`bindings/util/tdoa_replay.py`)

A small class (lists + append methods + counters), passed as optional
`recorder=None` to `replay()` and `EstimatorKalmanEmulator`:

- **`tdoa_records`** — one dict per TDoA sample reaching the estimator:
  `t_ms, idA, idB, distanceDiff, stdDev, innovation, innovation_var,
  accepted, filter_state`. `accepted` is `True`/`False` from the outlier
  filter, or `None` when the innovation is NaN (degenerate geometry — the
  firmware skips these before the filter; parity preserved). `filter_state`
  comes from a new optional `snapshot()` method on outlier filters, taken
  *after* the filter has processed the sample (so the record shows the state
  the decision produced).
- **`state_records`** — one dict per 1 kHz iteration:
  `t_ms, x, y, z, var_x, var_y, var_z` (via the covariance getter).
- **`n_skipped_unknown_anchor`** — count from `filter_known_anchors`, so views
  can report coverage honestly.

Constraint (documented + enforced): diagnostics require an *externalized*
outlier filter — the built-in C path hides the accept/reject decision. The
emulator raises `ValueError` when constructed with a recorder but no outlier
filter. `'integrator'` is the firmware-parity baseline, so nothing is lost.

### Filter `snapshot()`

`OutlierFilter.snapshot()` returns `{}` in the base class. `IntegratorFilter`
returns `{'integrator': ..., 'is_open': ...}` read from the C state struct
(fields are plain and SWIG-readable; verified). Experimental filters above the
line override as they like (pair_integrator: the current pair's entry;
mad_window: median/MAD/window fill).

## Phase 2 — truth (`bindings/util/tdoa_truth.py`)

Extends the module created in phase 0. Pure Python, no bindings import:

- `extract_ground_truth(log_data)`, `interp_ground_truth(gt, t_ms)` (moved in
  phase 0).
- `true_distance_diff(pos, pos_a, pos_b)` — the geometric truth
  `|pos − pos_b| − |pos − pos_a|`, matching the estimator convention
  (`distanceDiff = d(idB) − d(idA)`, see `_measurement` in
  `tdoa_selection.py`).
- `label_measurements(items, gt, anchor_positions)` — joins GT onto anything
  carrying `t_ms/idA/idB/distanceDiff` (candidate-group entries via a small
  adapter, or `tdoa_records` directly): adds `true_diff` and
  `true_error = distanceDiff − true_diff`; entries outside the GT time range
  or with unknown anchors get `None`. A separate helper
  `is_outlier(true_error, threshold)` keeps the labeling threshold explicit
  and out of the data.

Anchor positions here are plain `(x, y, z)` tuples/dicts so the module stays
importable without the SWIG bindings (mirrors the `tdoa_selection` approach);
a tiny adapter from `loco_utils` vec3 lives with the callers.

## Phase 3 — views (`tools/tdoa_plot/analyze_tdoa.py`)

One CLI, subcommand per view (Q1–Q8 table above). Shared glue module (log
loading, bindings bootstrap, replay-with-recorder, truth labeling) factored
out of `plot_tdoa.py` so both CLIs use it; `plot_tdoa.py` itself stays as the
trajectory view. Numeric analysis (regret, confusion matrix, DOP) lives here.

Common flags mirror `plot_tdoa.py`: `--anchors`, `--selection-policy`,
`--outlier-filter`, `--tdoa-model`, `--tdoa-std`, `--save`,
`--rebuild-bindings`. Views render interactive matplotlib or `--save` PNG.

Per-view sketches (enough to build from later; layout freedom intended):

- **`measurement-error` (Q1):** per-pair GT-referenced error time series
  (small multiples or selectable pairs); per-anchor aggregate (box/violin or
  bias±std bars — a bad anchor shows in all its pairs); error histogram/CDF;
  optional error-vs-XY-position scatter for multipath geography.
- **`selection` (Q2):** per-group |true_error| of the selected candidate vs
  the oracle-best candidate; cumulative regret over time; summary stats per
  selection policy (policies applied offline to the same groups — no replay).
- **`health` (Q3):** per-anchor packet rate over time; group-size histogram;
  gaps between accepted measurements; recorder skip counts when available.
- **`filter-decisions` (Q4):** innovation vs time, colored by
  accepted/rejected, GT true_error overlay; filter internal state (e.g.
  integrator level, open/closed shading) on a twin axis; confusion matrix
  (accepted/rejected × good/bad by GT threshold) with rates per anchor pair.
- **`consistency` (Q5):** per-axis estimate−GT error with ±kσ envelope from
  `state_records`; NIS = innovation²/S time series with χ²(1) bands and
  histogram; headline fraction-inside-envelope numbers.
- **`geometry` (Q7):** TDoA position-DOP at the interpolated GT position,
  computed from the pairs *accepted* in each time window (from
  `tdoa_records`); flags windows where geometry, not measurement quality,
  explains error.
- **`forensic` (Q6):** the composite — shared time axis stacking: 3D position
  error, acceptance rate + innovation magnitude, per-anchor packet rate,
  position σ. Exists to decompose any bad moment into measurement / filter /
  geometry / drought causes by eye.
- **`compare` (Q8):** the relocated `replay_tdoa` table (policy × filter ×
  model → rms/p95/max/fly-away), plus `--csv-dir` export, unchanged in
  substance.

## Testing

`test_python/` conventions; pure-Python tests run without bindings,
bindings-dependent tests `pytest.importorskip('cffirmware')`:

- **Phase 0:** existing suites keep passing after the moves (imports
  retargeted); resolver tests (core + experimental names, unknown-name error
  listing both sets); `replay` accepts a filter instance.
- **Phase 1:** covariance getter — bounds (NaN outside), matches known
  initial `P` diagonal from `kalmanCoreDefaultParams`, symmetry after an
  update. Innovation variance — S ≥ R always; agrees with a hand-computed
  HᵀPH + R on a crafted state; NaN on degenerate geometry exactly when the
  innovation is NaN. Recorder — populated on the replay smoke fixture; counts
  consistent (records == samples fed); `accepted` mirrors filter decisions;
  ValueError without externalized filter.
- **Phase 2:** truth math against synthetic geometry with exact expected
  values; interpolation edge cases (out of range, single sample); labeling
  with unknown anchors / missing GT → `None`.
- **Phase 3:** smoke per subcommand with `--save` on synthetic/fixture data
  (pattern of `test_tdoa_plot.py`); confusion-matrix and regret math unit
  tests with hand-built records.
- **Final validation:** all subcommands on `run_validation.bin` +
  `anchors.yaml`; sanity-check the headline numbers (e.g. baseline confusion
  matrix, consistency envelope) against the known-good run.

## Error handling

- Missing GT: truth-dependent views exit with a clear message naming the
  missing log config entries; `filter-decisions` degrades (no labels, says so
  on the figure).
- NaN innovation: recorded with `accepted=None`, never fed to the filter
  (firmware parity); views count and display these.
- Unknown anchors: skipped with the existing warning; count exposed on the
  recorder and reported by views.
- Logs without candidates: same behavior as today (clear message, plot what
  exists where sensible).

## Out of scope

- Firmware/live changes beyond the two analysis helpers (no new logging, no
  flight-code changes).
- Serialization format for diagnostics (records are plain dicts; dumping to
  disk is trivial for whoever needs it later).
- Acting on what the views reveal (new filters/policies/tunings) — that is
  the *next* project, and the whole point.

## Upstreaming note

After this design, the upstream PR cut is mechanical: everything except
`tools/tdoa_plot/` (and the capture configs / README, which are arguably
mergeable too) is below the line. Experimental algorithms that win graduate
to the firmware; those that lose are deleted with the directory.
