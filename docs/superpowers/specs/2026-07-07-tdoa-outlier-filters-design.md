# Pluggable TDoA outlier filters for offline replay — design

- **Status:** approved 2026-07-07; amended same day to fold in the
  `mm_tdoa_robust` update method — amendment awaiting review
- **Branch:** `tdoa-candidate-logging`
- **Author:** Rik Bouwmeester (with Claude Code)
- **Date:** 2026-07-07
- **Builds on:** `2026-07-02-tdoa-candidate-replay-foundation-design.md`

---

## Context

The replay tooling already lets us swap the *anchor-pair selection policy*
(`--policy` / `--policies`, backed by `SelectionPolicy` +
`POLICY_FACTORIES` + `make_policy` in `bindings/util/tdoa_selection.py`) and
tune the measurement noise (`--tdoa-std`). The TDoA *outlier filter*, however,
is baked in: `kalmanCoreUpdateWithTdoa` (`src/modules/src/kalman_core/mm_tdoa.c`)
always calls `outlierFilterTdoaValidateIntegrator`, and its tuning constants
(integrator size 300 ms, trigger at 2.0·std, accept at 2.5·std) are
`static const` in `src/modules/src/outlierfilter/outlierFilterTdoa.c`.

Goal: make the outlier filter swappable in the replay exactly the way
selection policies are — including a roster of candidate future filter designs
to A/B against the firmware integrator on identical logged data.

The structural difference from policies: policies transform log data *before*
the estimator, while the outlier filter runs *inside* the C measurement update,
gated on the innovation (`error = measured − predicted`) which depends on live
Kalman state at update time. A seam is needed to externalize that decision.

## Decisions made

- **Seam approach: behavior-preserving firmware refactor** of `mm_tdoa.c`
  (chosen over bindings-only `%inline` duplication, which drifts, and a full
  Python port, which loses float32 bit-exactness and weakens A/B conclusions).
- **Filter roster: all six** — `integrator`, `none`, `sanity`, `fixed`,
  `mad_window`, `pair_integrator`.
- **Amendment (same day): `mm_tdoa_robust` in scope** as an orthogonal second
  dimension — the *TDoA update method* (`standard` | `robust`) — instead of
  deferred. Rationale: `kalmanCoreRobustUpdateWithTdoa` is already compiled
  into the bindings (`bindings/setup.py`, `cffirmware.i`), it *ignores* its
  `outlierFilterState` parameter (the M-estimator's Geman-McClure weighting is
  its outlier handling; on-drone `kalman.robustTdoa = 1` runs ungated), and
  its innovation is computed with the same measurement model as `mm_tdoa.c`,
  so the Task-1 seam gates it unchanged. No additional C work.
- **CLI shape for the update method: a single `--tdoa-update` switch per run**
  (default `standard`) in both tools — not a third grid axis.

## 1. Firmware seam (`mm_tdoa.c` / `mm_tdoa.h`)

Behavior-preserving refactor:

- The innovation/jacobian math moves to a static helper
  `measurementModel(coreData, tdoa, &error, h)` that returns `false` in the
  degenerate case (`d0` or `d1` is zero).
- `kalmanCoreUpdateWithTdoa` becomes: model → existing outlier filter (both the
  default integrator path and the `CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK`
  steps variant preserved verbatim) → `kalmanCoreScalarUpdate`. Same float32
  operations in the same order, so flight behavior is bit-identical.
- Two new public functions (declared in `mm_tdoa.h`, reachable through the
  existing `%include "mm_tdoa.h"` in `bindings/cffirmware.i`):
  - `float kalmanCoreTdoaInnovation(coreData, tdoa)` — returns `error`, or NaN
    when degenerate. Touches no filter state (today the filter is not consulted
    in the degenerate case either).
  - `void kalmanCoreUpdateWithTdoaUnfiltered(coreData, tdoa)` — model + scalar
    update, no gate; no-op when degenerate.

The Python-gated path computes the model twice (once for the innovation, once
inside the unfiltered update). That is deterministic and identical, so replay
results are unaffected; the cost is offline-only.

## 2. New module `bindings/util/tdoa_outlier.py`

Mirrors `tdoa_selection.py`:

- `OutlierFilter` base: `name`, `reset()`, `validate(tdoa, error, now_ms) -> bool`.
- `FILTER_FACTORIES` dict of factories (fresh state per replay run) and
  `make_outlier_filter(name)` with the same unknown-name `ValueError` style as
  `make_policy`.
- `validate` receives the `tdoaMeasurement_t`-shaped object (anchor ids and
  positions, `distanceDiff`, `stdDev`), so pure-Python filters are unit-testable
  with stub objects and the module imports `cffirmware` only lazily (only
  `integrator` needs it) — same convention as `tdoa_replay.py`.

Roster:

| name | idea |
|---|---|
| `integrator` | The real C filter: own `OutlierFilterTdoaState_t`, calls the already-exposed `outlierFilterTdoaValidateIntegrator`. Firmware-parity baseline. |
| `none` | Accepts everything — control group; shows what filtering buys. |
| `sanity` | Only the physically-impossible check: `distanceDiff² < |pA − pB|²`. |
| `fixed` | Static gate `|error| < 2.5·stdDev` — no adaptivity; tests whether the integrator's adaptivity earns its keep. |
| `mad_window` | Sliding window (default 50) of recent innovations; reject when `|error − median| > k·MAD` (default k = 5). MAD is floored at `0.5·stdDev` so a quiet window cannot collapse the gate, and the filter accepts everything until the window holds 20 samples so it cannot lock itself out at startup. Accepted *and* rejected innovations enter the window (a rejected-but-real position jump must still be able to drag the median). |
| `pair_integrator` | Python port of the integrator logic with one state per `(idA, idB)` pair, so a single bad anchor cannot force the global filter open. |

## 3. Emulator + replay wiring

- `EstimatorKalmanEmulator(anchor_positions, outlier_filter=None)`:
  - `None` → today's `kalmanCoreUpdateWithTdoa` path, byte-for-byte unchanged
    for all existing users.
  - Filter object → `error = kalmanCoreTdoaInnovation(...)`; skip on NaN;
    `if filter.validate(tdoa, error, now_ms): kalmanCoreUpdateWithTdoaUnfiltered(...)`.
  - `filter.reset()` is called in `_lazy_init` (alongside
    `outlierFilterTdoaReset` today).
- `tdoa_replay.replay(...)` grows `params['outlier_filter']` (a name string,
  resolved via `make_outlier_filter`). Absent → legacy path (`None`), which is
  semantically the integrator.
- **Update method (amendment):** the emulator also takes
  `tdoa_update='standard' | 'robust'` (invalid value → `ValueError` at
  construction) and `replay` takes `params['tdoa_update']` (default
  `standard`). The four combinations of (filter, update):
  - no filter + `standard` → `kalmanCoreUpdateWithTdoa` (today's path, unchanged);
  - no filter + `robust` → `kalmanCoreRobustUpdateWithTdoa` directly —
    matches on-drone `robustTdoa = 1` (which is ungated);
  - filter + `standard` → innovation → `validate` → `kalmanCoreUpdateWithTdoaUnfiltered`;
  - filter + `robust` → innovation → `validate` → `kalmanCoreRobustUpdateWithTdoa`
    (a novel gated-robust combination, only available offline).
  Note: `mm_tdoa_robust.c` keeps state in `static` locals that carry over
  between calls (same on the drone) — replay stays deterministic for a given
  sample sequence, but robust results depend on call history within a run.

## 4. CLI

- `tools/tdoa_plot/plot_tdoa.py`: `--outlier-filter NAME` (default
  `integrator`); the chosen filter appears in the console printout and the plot
  title next to the policy.
- `tools/usdlog/replay_tdoa.py`: `--outlier-filters NAME...` (default
  `['integrator']`); evaluation grid is policies × filters with rows labeled
  `policy/filter`. Defaults reproduce today's output exactly.
- Both tools: `--tdoa-update {standard,robust}` (default `standard`), one
  value per run applied to the whole grid; echoed in the console output and
  the plot title.

## 5. Testing & validation

- **Parity test (the drift alarm):** replaying the same input with
  `outlier_filter=None` (legacy C path) vs `'integrator'` (new seam) must
  produce *exactly equal* trajectories. Guards both the firmware refactor and
  the seam wiring.
- **Bindings-free unit tests** for the pure-Python filters (`sanity`, `fixed`,
  `mad_window`, `pair_integrator`, `none`) using stub tdoa objects: gate
  thresholds, warmup, MAD floor, per-pair isolation, reset behavior.
- **Robust-path equivalence test (amendment):** replaying with
  `tdoa_update='robust'` and filter `'none'` must exactly equal the direct
  (no-filter) robust path — the innovation call is pure and `none` accepts
  everything, so the seam must be a no-op wrapper there.
- **Hardware-validated run:** replay `run_validation.bin` across the filter
  roster (and a `--tdoa-update robust` row) and record the accuracy table in
  the docs, like the F1 validation runs.
- `tools/usdlog/README_tdoa_candidates.md` gains an outlier-filter section next
  to the selection-policy one (roster, defaults, how to add a filter).

## Error handling

- Unknown filter name → `ValueError` listing available names (as `make_policy`).
- Degenerate geometry (NaN innovation) → sample skipped, filter state untouched
  (matches firmware).
- Filters must be total: `validate` returns a bool for any finite `error`; no
  exceptions in the steady state.

## Explicitly deferred (future work)

- Per-filter CLI parameters (e.g. `fixed:k=3` syntax) — constructor defaults
  only for now.
- Mahalanobis/chi-squared gate — needs the innovation variance `H·P·Hᵀ + R`
  exposed from C.
- Firmware-side adoption of any winning filter — this work is offline-only
  except for the behavior-preserving `mm_tdoa.c` refactor.
- A `--tdoa-updates` grid axis in `replay_tdoa.py` (single switch chosen
  instead; revisit if cross-method tables become a frequent need).
