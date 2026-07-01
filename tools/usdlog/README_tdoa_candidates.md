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
- Param `tdoaEngine.logCand` (default `0` = off). Set to `N` to log up to `N`
  candidates per packet. Doubles as a rate cap; set to `16` to log all.
- Build fragment `configs/tdoa3_lh_groundtruth.conf` (TDoA3 + Lighthouse as
  ground truth + uSD).

Host tooling (`tools/usdlog/`):
- `config_tdoa_candidates.txt` / `config_tdoa_candidates_lean.txt` — uSD configs.
- `read_usd_log.sh` — pull the log file off the drone via `cfcli` (raw memory).
- `anchors_from_cfcli.py` — turn `cfcli loco display` into an anchors YAML.
- `replay_tdoa.py` — replay + score policies vs ground truth.
- `bindings/util/tdoa_selection.py` — candidate grouping + selection policies.

## One-time setup

**1. Build the firmware**

```
make cf2_defconfig
./scripts/kconfig/merge_config.sh -m -O build build/.config configs/tdoa3_lh_groundtruth.conf
make olddefconfig
make -j$(nproc)          # -> build/cf2.bin
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

**4. Export anchor positions** (once, over USB or radio — needed for replay):

```
cfcli -u radio://0/100/2M/F00D2BEFED --csv loco display \
    | python3 -m tools.usdlog.anchors_from_cfcli > anchors.yaml
```

## Per-run loop

1. **Enable candidate logging** and pick the matching algorithm to baseline
   against (over a brief connection, before the run):

   ```
   cfcli -u radio://0/100/2M/F00D2BEFED param set tdoaEngine.logCand 16
   cfcli -u radio://0/100/2M/F00D2BEFED param set usd.logging 1
   ```

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
   python3 -m tools.usdlog.replay_tdoa run01.bin --anchors anchors.yaml \
       --policies baseline all median round_robin --csv-dir replay_out
   ```

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

## Watch for dropped uSD events (important)

The uSD ring buffer silently drops events if the SD write task can't keep up,
which breaks lossless replay. Candidate logging multiplies the event rate, so
verify no drops after a run by logging the `usd` group and comparing
`usd.eventsRequested` vs `usd.eventsWritten` (and watching `usd.fatWrBps`). If
you see drops:
- lower `tdoaEngine.logCand`,
- increase the buffer size (line 2 of `config.txt`),
- or switch to `config_tdoa_candidates_lean.txt`.

## Notes / limitations

- The replay uses the real `kalman_core`, `outlierFilterTdoa` and `mm_tdoa` via
  the `cffirmware` SWIG bindings (build them first; see `build_python.sh`).
- `replay_tdoa.py --tdoa-std` sets the measurement std dev used in the Kalman
  update. The firmware default is `0.15` (`0.30` with `DECK_LOCO_LONGER_RANGE`);
  the loco tag can further override it at runtime — match it for faithful replay.
- The firmware "youngest" matcher can't be reproduced exactly offline (it needs
  per-candidate last-update time, which is not in the event); `round_robin` and
  the data-driven policies above are the offline equivalents.
