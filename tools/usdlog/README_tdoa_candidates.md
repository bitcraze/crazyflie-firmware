# TDoA candidate logging + offline replay

Tooling to debug intermittent TDoA "fly-away" events (the estimate jumps ~1 m to
a wrong convergence point and snaps back) and to improve the anchor-pair
selection policy.

The idea: log TDoA data **as early as possible** — at the anchor-pair matching
stage in `tdoaEngine.c`, *before* selection collapses each received packet to a
single pair — so every candidate pair is captured. Record it to the uSD deck,
with Lighthouse crossing-beam position as ground truth on the same clock. Replay
offline through the *real* firmware Kalman core to locate outliers and A/B
different selection policies on identical data.

## What was added

Firmware:
- `EVENTTRIGGER(estTdoaCand, uint32 group, uint8 idA, uint8 idB, float distanceDiff, uint8 isSelected)`
  emitted at the matching stage in `src/utils/src/tdoa/tdoaEngine.c`. All
  candidates of one received packet share the same `group` value.
- Param `tdoaEngine.logCand` (default `0` = off). Set to any non-zero value
  (use `1`) to log **all** valid candidate pairs of every processed packet.
  There is no partial mode: the log is always a complete record of what the
  matcher saw, so the selected pair is always included.
- Build fragment `configs/tdoa3_lh_groundtruth.conf` (TDoA3 + Lighthouse as
  ground truth + uSD).

Host tooling (`tools/usdlog/`):
- `config_tdoa_candidates.txt` / `config_tdoa_candidates_lean.txt` — uSD configs.
- `read_usd_log.sh` — pull the log file off the drone via `cfcli` (raw memory).
- `anchors_from_cfcli.py` — turn `cfcli loco display` into an anchors YAML.
- `replay_tdoa.py` — replay + score policies vs ground truth.
- `bindings/util/tdoa_selection.py` — candidate grouping + selection policies.

## One-time setup

**0. Prerequisites**

- Hardware: Crazyflie with loco + Lighthouse + uSD decks stacked. The loco
  deck **must be modified for the alternative IRQ/RESET pins (IO_2/IO_3)** —
  the build config sets `CONFIG_DECK_LOCODECK_USE_ALT_PINS` because the
  default pins (RX1/TX1) collide with the Lighthouse UART. Symptoms of
  getting this wrong: unmodified deck + this config = loco never receives
  packets; default pins + Lighthouse stacked = watchdog reset ~5 s after
  every boot, right when the LH FPGA starts streaming.
- Host tools: `arm-none-eabi-gcc`, `cfcli` and `crazyflie-agent-cli`
  (`cargo install`, see their repos), and the Python firmware bindings:
  `make bindings_python` (see `build_python.sh`; replay imports `cffirmware`
  via `PYTHONPATH=build`).

**1. Build the firmware**

```
make cf2_defconfig       # Crazyflie 2.x; use cf21bl_defconfig for the 2.1 Brushless
./scripts/kconfig/merge_config.sh -m -O build build/.config configs/tdoa3_lh_groundtruth.conf
make olddefconfig
make -j$(nproc)          # -> build/cf2.bin (cf21bl.bin for the Brushless)
```

> Ground-truth mode means Lighthouse does **not** correct the estimator; the
> estimator runs on TDoA only, so fly-aways reproduce (and, in flight, are not
> caught by Lighthouse).

**2. Flash** (over radio or USB):

```
crazyflie-agent-cli flash --uri radio://0/100/2M/F00D2BEFED build/cf2.bin
# or: cfcli -u radio://0/100/2M/F00D2BEFED bootload ...
```

**3. Put the uSD config on the card** (physical, once — the card config cannot be
written over the air). Copy `config_tdoa_candidates.txt` to the card root as
`config.txt`.

**4. Export anchor positions** (once, over USB or radio, with the drone inside
loco coverage so the anchors report as valid — needed for replay):

```
cfcli -u radio://0/100/2M/F00D2BEFED loco display \
    | python3 -m tools.usdlog.anchors_from_cfcli > anchors.yaml
```

## Per-run loop

1. **Enable candidate logging** and pick the matching algorithm to baseline
   against (over a brief connection, before the run):

   ```
   cfcli -u radio://0/100/2M/F00D2BEFED param set tdoaEngine.logCand=1,usd.logging=1
   ```

   (cfcli takes comma-separated `name=value` pairs; `param set name 1` is a
   syntax error.)

   (Or set `usd` `enable on startup` to `1` in `config.txt` and skip the second
   command. `tdoaEngine.logCand` is not persistent; a persistent param or an
   app can set it on boot if you want fully radio-free runs.)

2. **Disconnect the radio** and fly / move the drone in loco + Lighthouse
   coverage. A radio link degrades Loco performance, so keep it off during the
   run.

3. **Read the log back** afterwards (radio or USB):

   ```
   tools/usdlog/read_usd_log.sh radio://0/100/2M/F00D2BEFED run01.bin
   ```

   The script stops logging first (so the file becomes readable), finds the
   microSD file size, and reads it into `run01.bin`.

4. **Replay + A/B the selection policies**:

   ```
   PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run01.bin --anchors anchors.yaml \
       --policies baseline all median round_robin --csv-dir replay_out
   ```

   (`PYTHONPATH=build` is where `make bindings_python` puts the `cffirmware`
   module; without it the import fails.)

   Output is a per-policy table of position error vs ground truth (rms / p95 /
   max / fraction above the fly-away threshold, plus the timestamp of the worst
   fly-away). Per-policy CSVs (`replay_out/replay_<policy>.csv`) hold the full
   trajectory + error series for plotting.

## Selection policies

- `baseline` — the pair the live firmware actually selected (`isSelected`);
  reproduces on-drone behaviour.
- `all` — feed **every** candidate to the estimator (the Kalman TDoA outlier
  filter still gates each update) instead of discarding all but one.
- `median` — the single candidate closest to the group's median `distanceDiff`
  (cheap consensus / outlier rejection).
- `round_robin` — one candidate per packet, rotating (deterministic replay of
  the "Random" matcher spirit).

Add your own in `bindings/util/tdoa_selection.py` (subclass `SelectionPolicy`).

## Dropped uSD events and SD write failures are checked automatically

The uSD ring buffer silently drops events if the SD write task can't keep up,
which breaks lossless replay. The firmware counts `usd.eventsRequested` vs
`usd.eventsWritten` (log variables, reset at each log start), and
`read_usd_log.sh` compares them after stopping the log on every readback:
it fails loudly (exit 1) if any events were dropped.

If drops are reported:
- increase the ring buffer size (line 2 of `config.txt`), or
- switch to `config_tdoa_candidates_lean.txt`.

A failing SD *write* (e.g. flaky card contact while the drone is handled)
silently stops logging mid-run: the drop counters freeze equal (so the drop
check alone would pass) and the log file is truncated or 0 bytes. The
firmware stores the first failing FatFS code in the sticky `usd.writeError`
log variable (0 = ok, reset at log start) and `read_usd_log.sh` fails the
readback if it is non-zero. If it triggers, reseat the microSD card
(power off, firm push-click) and check the card's health before retrying.

Do not use a log with drops or write failures for replay conclusions; the
holes are unflagged.

## Hardware validation checklist

Run this end-to-end check whenever the capture/readback/replay chain changes.
Collected logs stay local (do not commit them).

1. **Build** with the ground-truth config (see One-time setup) and **flash**.
2. **Arm logging**: `cfcli ... param set tdoaEngine.logCand=1,usd.logging=1`.
3. **Collect**: disconnect the radio; move the drone by hand (or fly, with
   explicit permission) for at least 60 s inside loco + Lighthouse coverage.
4. **Read back**: `tools/usdlog/read_usd_log.sh <uri> run.bin`
   - PASS requires: the automatic checks print `OK: no dropped events` and
     `OK: no SD write failures` (R1), and the file decodes
     (`replay_tdoa.py` reads it) with a plausible size (R2).
   - Radio readback of multi-MB logs is slow; pulling the SD card or using a
     `usb://` URI is much faster. The integrity checks still need the radio
     (or USB) link once, before removing the card.
5. **Verify capture faithfulness**:
   `PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run.bin --anchors anchors.yaml --policies baseline`
   - PASS requires: `baseline reconstruction: OK (... 0 unmatched live
     measurements ...)` — every logged `estTDOA` entry is explained by a
     selected candidate, in order (F1, measurement level). Position-gated
     extra candidates (`n_unmatched_baseline > 0`) are reported alongside and
     are expected, not a failure.
6. **Quantify replay fidelity**: the same command prints
   `baseline vs live stateEstimate: rms ... m, max ... m`.
   - Documented tolerance (F1, trajectory level), regime-dependent — treat
     regressions beyond these as failures:
     - **stationary** (drone at rest in coverage): **rms ≤ 0.10 m**
       (observed 0.044 m).
     - **hand-moved** (carried by hand; UWB antenna shadowing makes the TDoA
       stream genuinely noisy and the onboard estimator resets): **rms ≤
       1.0 m** (observed 0.67 m). For context, the *live* estimate itself was
       rms 0.45 m from the Lighthouse ground truth on the validated run, so
       replay error is of the same order as live error in this regime. The
       strict faithfulness claim is the measurement-level reconstruction in
       step 5, which must always be exact.

Latest validated run: 2026-07-07, firmware `4379d0e2` (cf21bl +
`tdoa3_lh_groundtruth.conf`), hand-moved ~75 s, 16121/16121 candidates
reconstructed, replay-vs-live rms 0.672 m / max 2.92 m, live-vs-groundtruth
rms 0.451 m / max 1.97 m. Stationary reference (same firmware family,
20 min desk log): 83398/83398 reconstructed, replay-vs-live rms 0.044 m.

## Notes / limitations

- The replay uses the real `kalman_core`, `outlierFilterTdoa` and `mm_tdoa` via
  the `cffirmware` SWIG bindings (build them first; see `build_python.sh`).
- `replay_tdoa.py --tdoa-std` sets the measurement std dev used in the Kalman
  update. The firmware default is `0.15` (`0.30` with `DECK_LOCO_LONGER_RANGE`);
  the loco tag can further override it at runtime — match it for faithful replay.
- The firmware "youngest" matcher can't be reproduced exactly offline (it needs
  per-candidate last-update time, which is not in the event); `round_robin` and
  the data-driven policies above are the offline equivalents.
- Candidate logging is gated only on clock correction, but the live estimator
  measurement (`estTDOA`) additionally requires both anchor positions to be
  valid (and the loco deck enabled). Two consequences: (a) selected candidates
  without a matching `estTDOA` entry are normal -- the drop-check and
  `verify_baseline_reconstruction` account for this (`n_unmatched_baseline`
  can be > 0; only `n_unmatched_est` gates PASS/FAIL); (b) replay feeds the
  Kalman core some measurements the on-drone estimator never consumed, which
  is a known, bounded fidelity limitation quantified by the step-6 divergence
  numbers below.
