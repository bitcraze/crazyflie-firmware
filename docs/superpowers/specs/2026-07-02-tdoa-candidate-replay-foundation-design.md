# TDoA candidate capture + offline-replay foundation — design

- **Status:** in review — spec re-reviewed against the code and open decisions resolved 2026-07-06; not yet approved by user
- **Branch:** `tdoa-candidate-logging`
- **Author:** Rik Bouwmeester (with Claude Code)
- **Date:** 2026-07-02
- **Related commit (existing draft):** `d0bb7f61` "Add TDoA anchor-pair candidate logging + offline replay tooling"

---

## Context

The existing branch was written vibe-first and now needs to be turned into a
verifiable spec + plan. This document reverse-engineers the *intended* behavior,
reframes the goal, and — most importantly — defines what "correct" means so the
implementation plan can turn that into tests and fixes.

The live TDoA pipeline (`src/utils/src/tdoa/tdoaEngine.c`) collapses each
received packet to a single anchor pair at the matching stage
(`findSuitableAnchor`), discarding the other valid candidate pairs. That
discarded data is exactly what is needed to (a) understand why the estimate
misbehaves at specific moments and (b) test whether a different use of the
candidates would do better — but it never leaves the drone.

## Goal (Section 1)

Deliver a **verified TDoA capture-and-replay foundation**:

- **Faithful capture** — the logged `estTdoaCand` stream is a correct, complete,
  unambiguous record of what the live matcher saw each packet.
- **Reliable readback** — getting that log off the drone is deterministic and
  losslessness is checkable.
- **Faithful replay** — running the logged samples through the *real* firmware
  Kalman core (via the `cffirmware` SWIG bindings) reproduces on-drone behavior
  closely enough to trust as a baseline. Bit-identical is an aspiration, not a
  requirement (see F1).
- **Clean analysis seam** — a small, documented, tested Python interface
  (`log → candidate groups → measurements → estimator run`) that arbitrary
  analysis scripts build on.

**The foundation is the deliverable.** Downstream analysis capabilities
(selection-strategy comparison, forensic drill-down into bad moments, offline
parameter tuning) are easy to write as scripts *once the foundation is
trustworthy*, and are explicitly **not** where energy is spent now. The existing
selection policies + scoring stay only as a **thin reference example** that
exercises the seam.

## Components (existing baseline — Section 2)

| Layer | Piece | Role |
|---|---|---|
| Firmware | `tdoaEngine.c` `logTdoaCandidates()` + `EVENTTRIGGER(estTdoaCand, …)` | Emit up to `logCand` candidate pairs/packet at the matching stage |
| Firmware | param `tdoaEngine.logCand`, state `candidateLogMaxCount` / `candidateLogGroup` | Enable gate + per-packet grouping counter (draft also truncates per packet; truncation is removed by C1) |
| Build | `configs/tdoa3_lh_groundtruth.conf` | TDoA3 + Lighthouse-as-groundtruth + uSD |
| Record | `tools/usdlog/config_tdoa_candidates{,_lean}.txt` | uSD what-to-log (candidates + IMU + ground truth + baseline `estTDOA`) |
| Host | `bindings/util/tdoa_selection.py` | Regroup candidates per packet; pluggable selection policies |
| Host | `tools/usdlog/replay_tdoa.py` | Replay through the Kalman core; score vs ground truth |
| Plumbing | `tools/usdlog/read_usd_log.sh` | Pull the log off the drone over CRTP (memory subsystem) |
| Plumbing | `tools/usdlog/anchors_from_cfcli.py` | Export anchor positions → anchors YAML |

The `estTdoaCand` event payload is
`(uint32 group, uint8 idA, uint8 idB, float distanceDiff, uint8 isSelected)`.

Dependencies that already exist and are relied on:
`bindings/util/estimator_kalman_emulator.py` (has an `estTDOA` path and
`run_one_1khz_iteration`), `bindings/util/loco_utils.py`
(`read_loco_anchor_positions`), `tools/usdlog/cfusdlog.py` (`decode`), and the
`tdoaStorage*` accessors used by `logTdoaCandidates`.

## What "correct foundation" means (Section 3 — the real work)

Each item below becomes a test and, where it fails, a fix in the plan.

### Capture faithfulness (firmware)

- **C1 — Groups are never truncated (decided 2026-07-06).** `logCand` becomes a
  plain enable flag: 0 = off, any non-zero value = emit *every* valid candidate
  pair. The draft's per-packet truncation (stop after `logCand` events, walking
  the remote list in storage order while the matcher picks from a rotating
  offset) could silently drop the live-selected pair, leaving a group with no
  `isSelected` candidate and a corrupt baseline for that packet. The cap was an
  arbitrary bandwidth guess, not a requirement. **Fix:** drop the
  `loggedCount < candidateLogMaxCount` loop condition in `logTdoaCandidates` and
  rewrite the `logCand` param docs (including removing the incorrect claim that
  drops can be checked via existing `usd.*` log variables — see R1). With no
  truncation, the selected pair is present by construction whenever the matcher
  selected one; uSD bandwidth pressure is handled by detection (R1), not by
  truncation.
- **C2 — Candidate set = matcher's feasible set.** The validity predicate in
  `logTdoaCandidates` (`tdoaStorageGetRemoteTimeOfFlight != 0` + cached seqNr
  match) must match exactly what the real matchers (`matchRandomAnchor`,
  `matchYoungestAnchor`) consider selectable — no phantom pairs the firmware
  could never pick, and no missing feasible pairs. Any intentional difference
  must be documented. **Scope note:** in hybrid mode
  (`CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE`) the matcher excludes an anchor id that
  `logTdoaCandidates` does not know about, so parity does not hold there; C2 is
  scoped to the non-hybrid build used by this project
  (`tdoa3_lh_groundtruth.conf`) and hybrid mode is documented as out of scope.
  A group may also legitimately lack an `isSelected` candidate (e.g. no
  matching algorithm configured → `selectedId = 0xFF`); the offline baseline
  policy must skip and count such groups rather than fail.
- **C3 — Grouping is exact.** Every emitted event of one packet shares one
  `group` value; groups never merge or split across packets. The
  `candidateLogGroup` counter increments once per enabled, clock-correction-gated
  packet (even when zero candidates are emitted) — confirm this cannot cause
  offline mis-grouping.
- **C4 — Logging never mutates live state.** `logTdoaCandidates` currently uses
  `tdoaStorageGetCreateAnchorCtx`, which on a miss *creates* a context slot by
  evicting (memset) the oldest stored anchor — so enabling logging can perturb
  the very pipeline being observed (bites when active anchors exceed
  `ANCHOR_STORAGE_COUNT`). **Fix:** use the non-creating
  `tdoaStorageGetAnchorCtx` in the logging path. This strengthens the non-goal
  below: capture must be side-effect-free on the live estimator when enabled,
  not only when disabled.

### Readback reliability (plumbing)

- **R1 — Losslessness is checked automatically (decided 2026-07-06).** The uSD
  ring buffer silently drops events if the SD write task cannot keep up. The
  firmware already counts `eventsRequested` vs `eventsWritten` (reset at each
  log start), but today these are only DEBUG_PRINTed to the console at log stop
  — they are **not** exposed as log variables (the `usd` log group only carries
  byte-rate stats), so the check is currently impossible over the log framework.
  **Fix:** expose both counters as `usd.*` log variables, and make
  `read_usd_log.sh` (already connected to the drone at readback time) read them
  after stopping the log and fail loudly on mismatch. Losslessness is
  machine-checked on every readback rather than left to a manual procedure —
  which matters more now that C1 removes the bandwidth cap, making detection the
  only guard against silent holes.
- **R2 — `read_usd_log.sh` size/offset logic is correct** against a real card
  (stop-logging → find MicroSD size → `mem read` at offset 0, full length).

### Replay fidelity (bindings)

- **F1 — Baseline replay ≈ on-drone.** The `baseline` policy reconstructs the
  selected measurement from the candidate stream; this should equal the logged
  `estTDOA` (same `calcDistanceDiff` inputs → aim for bit-identical measurement
  values, which is realistic). The replayed *trajectory* should track the live
  `stateEstimate` within a **documented tolerance**; exact match is an
  aspiration, not a gate. Known divergence sources to characterize: estimator
  initialization, timestamp quantization (see F2), uSD drops, runtime
  measurement-std overrides by the loco tag vs the replay `--tdoa-std`, and the
  anchor-position gate (the live estimator only consumes measurements whose
  anchor positions are valid; replay may feed the Kalman core additional
  position-gated candidates the on-drone estimator never saw).
- **F2 — Sample ordering / timestamp model is sound.** uSD stamps events at
  write time, so replay uses the group's *min* timestamp for all candidates of a
  packet. Confirm this heuristic plus the IMU/TDOA interleaving produces a
  correctly ordered sample stream into the emulator.

## The analysis seam (Section 4 — must be clean)

A small, stable, documented, tested Python interface so future analysis scripts
are trivial to write:

- **`build_candidate_groups(log_data) → [group]`** — regroup `estTdoaCand`
  events per packet. Exists; needs tests and a documented, frozen group schema
  (`group`, `t_ms`, `idA`, `candidates:[{idB, distanceDiff, isSelected}]`).
- **A replay entry point** — e.g. `replay(anchor_positions, imu_samples,
  tdoa_samples, params) → trajectory`, factored out of `replay_tdoa.py`'s
  `run_policy` so scripts call it directly without going through the CLI. Tuning
  params (currently `tdoa_std`; later outlier-filter settings) passed as a plain
  dict/object — this is the seam that a future "param tuning" script uses.
- **A selection-policy protocol** — `group → [measurement]`, already present in
  `tdoa_selection.py`. Kept as-is and documented as the extension point.

`replay_tdoa.py` (scoring table, CSV) becomes a **thin reference consumer** of
the seam demonstrating end-to-end use — not where logic lives.

## Verification & testing strategy (Section 5)

Layered, hardware-light where possible:

1. **Python unit tests** (`test_python/`, pytest) — grouping (C3), baseline
   reconstruction equals logged `estTDOA` (F1 measurements), sample ordering
   (F2), ground-truth interpolation + scoring. Synthetic fixtures, no hardware.
2. **Firmware Ceedling test**
   (`test/utils/src/tdoa/test_tdoa_engine_candidates.c` or similar) — mocked
   `eventtrigger` / `tdoaStorage`, asserting C1 (no truncation — all valid
   candidates emitted, selected pair flagged), C2 (feasible-set parity), C3
   (grouping), C4 (no storage mutation), and the clock-correction gate. Run via
   `rake unit`. Follows the pattern of the existing
   `test/utils/src/tdoa/test_tdoa_storage.c`.
3. **Replay smoke test** — a tiny synthetic log through the real `cffirmware`
   bindings, proving the seam runs end-to-end (pattern of
   `test_python/test_kalman_core.py`).
4. **Hardware end-to-end validation** (the real proof, on the user's rig):
   build + flash `tdoa3_lh_groundtruth.conf`, set `logCand`, collect a real log
   (movement or flight — **flight only with the user's explicit go-ahead each
   time**), read it back, verify R1 losslessness + R2 readback, then replay and
   compare the baseline trajectory against the on-drone `stateEstimate` to
   quantify F1 divergence. Result: a documented checklist. Hardware logs stay
   **local** for now — committing one as a replay fixture (following the
   `test_python/fixtures/kalman_core` precedent) was considered and deferred
   (decided 2026-07-06).

### Hardware loop roles

- Claude drives: build, flash (`crazyflie-agent-cli` / `cfcli`), param set,
  log readback, replay.
- Claude may command autonomous flight (takeoff / trajectory / land) **only with
  the user's explicit permission each time**.
- User physically places the drone in loco + Lighthouse coverage. The SD-card
  deck config is already written to the card.

## Non-goals (Section 6)

- Actually diagnosing or fixing fly-aways, or judging which selection policy is
  "best."
- Building out forensics / parameter-tuning as features (only the *seam* that
  would support them).
- New selection policies beyond the existing four
  (`baseline`, `all`, `median`, `round_robin`).
- Any change to the live estimator's behavior. Candidate logging must be
  side-effect-free on the flight path both when disabled and when enabled (C4);
  the only permitted effect of enabling it is event emission.

## Resolved decisions (2026-07-06)

- **C1:** remove per-packet truncation entirely; `logCand` is an enable flag
  (0 = off, non-zero = log all valid candidates). The bandwidth cap was an
  arbitrary guess; completeness of the record wins, and R1 detection covers uSD
  pressure.
- **R1:** expose `eventsRequested` / `eventsWritten` as `usd.*` log variables
  and have `read_usd_log.sh` verify them automatically after every readback,
  failing loudly on drops.
- **Replay fixture:** hardware logs stay local for now; committing a fixture
  (per the `test_python/fixtures/kalman_core` precedent) is deferred and may be
  revisited once the foundation is proven.
