# TDoA candidate logging + offline replay

Tooling to investigate TDoA position errors — in particular intermittent
"fly-aways", where the estimate jumps ~1 m to a wrong convergence point and
snaps back — and to compare anchor-pair selection policies on real data.

The idea: log TDoA data **as early as possible**, at the anchor-pair matching
stage in `tdoaEngine.c`, *before* selection collapses each received packet to a
single pair. Every candidate pair is captured, so the log holds not just what
the estimator used but everything it could have used. Record it to the uSD deck
alongside an independent position reference on the same clock, then replay it
offline through the *real* firmware Kalman core to locate outliers and A/B
different policies on identical data.

## What the firmware provides

- `EVENTTRIGGER(estTdoaCand, uint32 group, uint8 idA, uint8 idB, float distanceDiff, uint8 isSelected)`,
  emitted at the matching stage in `src/utils/src/tdoa/tdoaEngine.c`. All
  candidates of one received packet share the same `group` value, and
  `isSelected` marks the one the live algorithm picked.
- Param `tdoaEngine.logCand`, default `0` (off). Any non-zero value logs **all**
  valid candidate pairs of every processed packet. There is no partial mode: the
  log is always a complete record of what the matcher saw, so the selected pair
  is always included.

## Files here

- `tdoa3_lh_groundtruth.conf` — build fragment for the rig this was developed
  on (TDoA3 + Lighthouse as ground truth + uSD).
- `config_tdoa_candidates.txt` / `config_tdoa_candidates_lean.txt` — uSD card
  configs. The lean one drops the live estimate and the baseline measurement,
  for when the full one overruns the ring buffer.
- `replay_tdoa.py` — replay a captured log and score policies against the
  reference.

The replay machinery itself lives in `bindings/util/`: `tdoa_replay.py` is the
interface, with `tdoa_selection.py`, `tdoa_outlier.py` and `tdoa_std.py` holding
the pluggable policies, filters and noise models.

## One-time setup

**0. Prerequisites**

- Hardware: a Crazyflie with Loco, Lighthouse and uSD decks stacked, if you use
  the reference rig. The Loco deck **must be modified for the alternative
  IRQ/RESET pins (IO_2/IO_3)** — `tdoa3_lh_groundtruth.conf` sets
  `CONFIG_DECK_LOCODECK_USE_ALT_PINS` because the default pins (RX1/TX1)
  collide with the Lighthouse UART. Symptoms of getting this wrong: an
  unmodified deck with this config never receives Loco packets; the default
  pins with Lighthouse stacked give a watchdog reset a few seconds after every
  boot, right when the Lighthouse FPGA starts streaming.
- Any reference source works as long as it lands in the same log on the same
  clock. Lighthouse in ground-truth mode is what the config fragment sets up;
  the replay tool reads `lighthouse.x/y/z`, so another source means adapting
  `extract_ground_truth` in `replay_tdoa.py`.
- Host: SWIG and the Python firmware bindings, built with `make
  bindings_python` (see `docs/building-and-flashing/build.md`). Replay
  imports `cffirmware` from `build/`.

**1. Build the firmware**

```
make cf2_defconfig       # or cf21bl_defconfig for the 2.1 Brushless
./scripts/kconfig/merge_config.sh -m -O build build/.config \
    tools/usdlog/tdoa3_lh_groundtruth.conf
make olddefconfig
make -j$(nproc)          # -> build/cf2.bin
```

> Ground-truth mode means Lighthouse does **not** correct the estimator. The
> estimate runs on TDoA alone, so its errors reproduce — and, in flight, are
> not caught by Lighthouse either. Fly accordingly.

**2. Flash** it over radio or USB with your usual tool.

**3. Put the uSD config on the card.** Copy `config_tdoa_candidates.txt` to the
card root as `config.txt`. This is physical and one-time; the card config cannot
be written over the air.

**4. Write an anchors YAML** for the replay, mapping anchor id to position:

```
0: {x: 3.0, y: 4.08, z: 0.2}
1: {x: 3.0, y: -4.08, z: 0.2}
```

Take the positions from your anchor survey, or read them off the drone (the
Loco Positioning tab in cfclient shows them). It has to match the system the
log was recorded in — replay geometry is only as good as this file.

## Per-run loop

1. **Arm logging** over a brief connection, before the run:

   ```
   tdoaEngine.logCand = 1
   usd.logging        = 1
   ```

   Set both with whatever you use to set params. `usd.logging` can also be
   armed from the card by setting *enable on startup* to `1` in `config.txt`.
   `tdoaEngine.logCand` is not persistent, so a fully link-free run needs a
   persistent param or an app to set it on boot.

2. **Disconnect the radio** and fly or move the drone inside Loco and reference
   coverage. A radio link degrades Loco performance, so keep it off during the
   run.

3. **Stop logging and read the log back.** Setting `usd.logging = 0` closes the
   file; until then the uSD memory reports a size of 0. Then either pull the
   card, or read the file over the link from the `MicroSD` memory. Before
   pulling the card, check the run is intact while you still have a link — see
   below.

4. **Replay and compare policies**:

   ```
   PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run01.bin \
       --anchors anchors.yaml \
       --policies baseline all median round_robin --csv-dir replay_out
   ```

   The output is a per-policy table of position error against the reference
   (rms / p95 / max / fraction above the fly-away threshold, plus the timestamp
   of the worst fly-away). With `--csv-dir`, each policy also gets a CSV holding
   the full trajectory and error series for plotting.

## Selection policies

- `baseline` — the pair the live firmware selected (`isSelected`); reproduces
  on-drone behaviour.
- `all` — feed **every** candidate to the estimator instead of discarding all
  but one. The outlier filter still gates each update.
- `median` — the single candidate closest to the group's median `distanceDiff`
  (cheap consensus).
- `round_robin` — one candidate per packet, rotating; a deterministic stand-in
  for the firmware's "youngest" matcher.

Add your own by subclassing `SelectionPolicy` in
`bindings/util/tdoa_selection.py`.

## Checking that a capture is intact

Replay conclusions are only worth as much as the log, and both ways a log can
go wrong are silent.

**Dropped events.** The uSD ring buffer drops events if the write task cannot
keep up, which puts unflagged holes in the log. The firmware counts
`usd.eventsRequested` against `usd.eventsAccepted` (log variables, reset at each
log start). They must be equal. If they are not, either raise the ring buffer
size on line 2 of `config.txt` or switch to `config_tdoa_candidates_lean.txt`.

**SD write failures.** A failing write — a flaky card contact while the drone is
handled, say — stops logging mid-run. The drop counters freeze equal, so the
drop check alone would pass, and the file is truncated or 0 bytes. The firmware
keeps the first failing FatFS result in `usd.writeError` (0 = ok, reset at log
start). If it is non-zero, reseat the card (power off, firm push-click) and
check the card's health before retrying.

Read both while the drone is still connected. They live on the drone, not in
the log, so pulling the card first throws them away.

**Capture faithfulness.** Every replay prints a reconstruction check:

```
baseline reconstruction: OK (16121/16121 live measurements matched, ...)
```

Every logged `estTDOA` entry must be explained by a selected candidate, in
order. Extra position-gated candidates are reported alongside and are expected,
not a failure (see the limitations below).

**Replay fidelity.** With the `baseline` policy the tool also prints how far the
replayed trajectory drifts from the live `stateEstimate`. This is
regime-dependent. On a drone at rest in coverage, expect rms well under 0.1 m
(0.044 m observed over a 20 min desk log). Hand-carrying is far harder — antenna
shadowing makes the TDoA stream genuinely noisy and the onboard estimator resets
— and rms below 1 m is normal there (0.67 m observed over ~75 s, against a live
estimate that was itself 0.43 m from ground truth). Treat this as a sanity
check, not a tolerance; the strict claim is the measurement-level
reconstruction above, which must always be exact.

## Notes and limitations

- Replay runs the real `kalman_core`, `outlierFilterTdoa` and `mm_tdoa` through
  the `cffirmware` SWIG bindings, not a reimplementation.
- `--tdoa-std` sets the measurement std dev used in the Kalman update. The
  firmware default is `0.15` (`0.30` with `DECK_LOCO_LONGER_RANGE`), and the
  Loco tag can override it at runtime — match it for faithful replay.
- The firmware's "youngest" matcher cannot be reproduced exactly offline: it
  needs a per-candidate last-update time that the event does not carry.
- Candidate logging is gated only on clock correction, while the live estimator
  measurement (`estTDOA`) additionally requires both anchor positions to be
  valid. Two consequences: selected candidates without a matching `estTDOA`
  entry are normal, and replay feeds the Kalman core some measurements the
  on-drone estimator never consumed. That is a known, bounded fidelity limit,
  and it is part of what the replay-vs-live divergence above measures.
