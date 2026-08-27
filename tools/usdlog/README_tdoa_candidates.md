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
- Param `tdoaEngine.logCand`. Any non-zero value logs **all** valid candidate
  pairs of every processed packet. There is no partial mode: the log is always a
  complete record of what the matcher saw, so the selected pair is always
  included. It initialises from `CONFIG_DECK_LOCO_TDOA3_LOG_CANDIDATES`, which
  `tdoa3_lh_groundtruth.conf` sets — so on a build made from that fragment it is
  **already on at boot** and needs no link. Upstream default is off.

## Files here

- `tdoa3_lh_groundtruth.conf` — build fragment for the rig this was developed
  on (TDoA3 + Lighthouse as ground truth + uSD).
- `config_tdoa_candidates.txt` / `config_tdoa_candidates_lean.txt` — uSD card
  configs. The lean one drops the live estimate and the baseline measurement,
  for when the full one overruns the ring buffer. The full one also subscribes
  to `on:tdoaFlightStep` so a capture flown by the optional onboard app says
  which sequence it was and where each step began; it is an event rather than a
  periodic variable, so it costs a few dozen records per run rather than
  bandwidth at the sample rate. The uSD deck skips events it does not know, so
  that line is harmless on a build without the app. The lean config leaves it
  out along with everything else non-essential.
- `read_usd_log.sh` — stop logging, run the integrity gates over the link and
  optionally pull the log off the drone. The entry point at the end of a run.
- `anchors_from_cfcli.py` — turn `cfcli loco display` output into the anchors
  YAML the replay needs.
- `replay_tdoa.py` — replay a captured log and score policies against the
  reference.
- `tdoa_experiment.py` — sweep a grid of (policy, outlier filter, TDoA model,
  std) on one log, scored on a train/validate time split so tuning cannot
  overfit the segment it was tuned on.

Plotting lives in `../tdoa_plot/plot_tdoa.py`, which draws the replayed
trajectory, the live on-drone estimate and the Lighthouse ground truth on the
same axes. It is a separate `uv` project because it needs matplotlib, which
nothing else here does.

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
- Host: the Rust `cfcli` on `PATH` (tested with 0.12.0). Every on-drone step
  below is a `cfcli` call, directly or through `read_usd_log.sh`. Point at a
  different binary with `CFCLI=/path/to/cfcli`.

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

Take the positions from your anchor survey, or read them off the drone, which
is what the drone is actually using and therefore the thing replay has to
agree with:

```
cfcli -u "$CF_URI" loco display | python3 -m tools.usdlog.anchors_from_cfcli > anchors.yaml
```

Inactive or invalid anchors are skipped. It has to match the system the log was
recorded in — replay geometry is only as good as this file, and a stale one
shows up as a replay that diverges from ground truth for no visible reason.

## Per-run loop

Every on-drone step is a `cfcli` call. Set the URI once; `cfcli scan --csv`
lists the drones the radio can see.

```
export CF_URI=radio://0/100/2M/F00D2BEFED
```

1. **Pre-flight, over a brief connection.** All four of these must read
   non-zero, or the run produces nothing usable:

   ```
   cfcli -u "$CF_URI" --csv param get \
       deck.bcLoco,deck.bcLighthouse4,deck.bcUSD,usd.canLog
   ```

   Each `deck.*` is non-zero once that deck's driver has initialised.
   `deck.bcLoco` reading 0 with this build almost always means an unmodified
   deck — see the prerequisites. `usd.canLog` is the one that is easy to
   overlook: it only goes non-zero after the card mounted *and* `config.txt`
   parsed without error, so it is the single check that the card config is
   actually in effect. A typo in `config.txt` shows up here and nowhere else
   until the log comes back missing the events you wanted.

2. **Arm logging**, on the same connection. **`usd.logging` must be the last
   thing you send, and nothing may follow it:**

   ```
   cfcli -u "$CF_URI" param set tdoaEngine.logCand=1   # not needed on a build from tdoa3_lh_groundtruth.conf
   # ... any other params for this run ...
   cfcli -u "$CF_URI" param set usd.logging=1      # LAST. Do not talk to it after this.
   ```

   Order is not stylistic. With `config.txt` subscribed to `estAcceleration`
   and `estGyroscope`, events fire per IMU sample at 1 kHz each, and
   `usddeckWriteEventData` calls `vTaskResume()` on the SD write task for every
   one of them. A CRTP packet arriving into that load can find
   `crtpPacketDelivery` already full, and `radiolink.c:171` asserts — observed
   in practice, and it takes the drone down hard enough that it stops answering
   the radio and needs a power cycle. Setting `usd.logging` last means no packet
   ever arrives during the loaded window.

   `tdoaEngine.logCand` is safe to set early: `usddeckWriteEventData` returns
   immediately while `enableLogging` is 0, so candidate events cost nothing
   until logging is actually on.

   Neither parameter carries `PARAM_PERSISTENT`, so `param set --store` cannot
   persist them; both live in RAM and survive the disconnect in the next step,
   but not a reboot or a battery swap. Both can instead be armed at boot, which
   is strictly better for capture work — see *Link-free capture* below.

3. **Disconnect the radio** and fly or move the drone inside Loco and reference
   coverage. A radio link degrades Loco performance, so keep it off during the
   run.

   Flying by hand or from a link-free onboard app are both fine here. If you
   want the latter, `examples/app_tdoa_flight/` flies predefined sequences on a
   trigger param and lands itself; see its README. It is optional and nothing
   in this directory depends on it: it never *starts* logging, so step 2 above
   is unchanged. It does clear `usd.logging` at touchdown (its
   `tdoaFlight.stopLog`, default on) so the file is closed even if the battery
   goes flat before you reconnect — step 4 sets it to 0 anyway, so this changes
   nothing you do here.

4. **Stop logging and verify the capture — while the drone is still powered,
   and before pulling the card:**

   ```
   tools/usdlog/read_usd_log.sh "$CF_URI" --check-only
   ```

   If you flew by hand, this connection lands on a drone that is *still
   logging* — the loaded state described in step 2. It is a single short
   command and it stops logging first, but if the drone asserts instead of
   answering, power cycle and read the card directly: the log is already on it,
   only the counters are lost. Flights run by the onboard app have already
   cleared `usd.logging` at touchdown, so this does not apply to them.

   This sets `usd.logging = 0`, which is what closes the file (until then the
   uSD memory reports a size of 0), runs all three integrity gates and prints
   the resulting log size. It transfers no log data, so it is fast even for a
   multi-MB capture. Only pull the card once it has passed: the counters it
   reads live in drone RAM and are gone after a power cycle, so pulling the
   card first throws away the only evidence that the log is intact. See
   *Checking that a capture is intact* below for how to read the result — the
   exit code alone is not sufficient.

   **Then pull the card and copy the file with a card reader.** Do *not* read
   the log over the air unless you are prepared to wait 30+ minutes for a
   typical log. The transfer holds the radio for its whole duration; a card
   reader does it in seconds. The over-the-air form is for when the drone is
   out of reach, not for routine use:

   ```
   tools/usdlog/read_usd_log.sh "$CF_URI" run01.bin                        # 30+ min
   tools/usdlog/read_usd_log.sh "$CF_URI" run01.bin --replay anchors.yaml
   ```

   `--check-only` transfers no log data and is unaffected by any of this; it
   is the only step that must happen over the link.

5. **Replay and compare policies**:

   ```
   PYTHONPATH=build python3 -m tools.usdlog.replay_tdoa run01.bin \
       --anchors anchors.yaml \
       --policies baseline all median round_robin --csv-dir replay_out
   ```

   The output is a per-policy table of position error against the reference
   (rms / p95 / max / fraction above the fly-away threshold, plus the timestamp
   of the worst fly-away). With `--csv-dir`, each policy also gets a CSV holding
   the full trajectory and error series for plotting.

## Link-free capture

Every failure mode in step 2 above comes from having a link up while the drone
is under event load. Both switches can be armed without one:

- **Candidate logging**: build with `CONFIG_DECK_LOCO_TDOA3_LOG_CANDIDATES=y`,
  which `tdoa3_lh_groundtruth.conf` already sets. `tdoaEngine.logCand` then
  initialises to 1 and is on from the first packet after boot. It stays writable
  at runtime, so a run can still turn it off.
- **uSD logging**: set *enable logging on startup* (line 4 of `config.txt`) to
  `1`. The card arms itself at mount time.

With both, a capture is: power on, fly, land, power off. No `param set` at any
point, so the `radiolink.c:171` assert cannot happen and the ordering rule in
step 2 stops mattering. Step 1's preflight check and step 4's integrity gates
are still worth a link — just not during the run.

The cost is that every boot writes a new log file, whether or not you wanted
one, and every boot burns a filename (`log00`, `log01`, …). For a session where
essentially every power cycle is a capture, that is the right trade; for a build
you also fly normally, it is not. That is why the upstream default of both is
off and only the capture fragment turns them on.

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

## Plotting and sweeping

`replay_tdoa.py` prints a scored table. To see the trajectories:

```
uv run --project tools/tdoa_plot tools/tdoa_plot/plot_tdoa.py \
    log02 --anchors anchors.yaml
```

To search combinations rather than compare a handful, `tdoa_experiment.py` takes
a YAML grid and scores every entry on a train/validate time split, so a setting
tuned on the first half is reported on the second:

```
- {policy: baseline, filter: integrator, model: standard, std: 0.15}
- {policy: median,   filter: mad_window, model: standard, std: 0.30}
```

```
python3 -m tools.usdlog.tdoa_experiment log02 \
    --anchors anchors.yaml --grid grid.yaml --out results.csv
```

Each replay runs in a fresh forked worker: the robust TDoA model keeps
M-estimation state in a process-wide static buffer, so runs must not share a
process.

## Checking that a capture is intact

Replay conclusions are only worth as much as the log, and both ways a log can
go wrong are silent.

`read_usd_log.sh` runs every gate below; the rest of this section is what it
checks and why. Its exit codes:

| exit | meaning |
| --- | --- |
| `0` | logging stopped, gates passed, file closed and non-empty |
| `1` | a gate failed — the message on stderr names which one |
| `2` | usage error (bad arguments) |

**Two failure paths are deliberately non-fatal, and automation has to handle
them.** If a counter cannot be read at all — firmware built without it, or a
link that drops the log subscription — the script warns and *continues*:

```
!! WARNING: could not read usd.eventsRequested/usd.eventsAccepted.
!! WARNING: could not read usd.writeError (firmware without it?).
```

Either line means that gate did not run, and the script can still exit `0` with
losslessness unverified. **Checking the exit status alone is not enough.**
Require both of these on stdout before trusting a capture:

```
>> OK: no dropped events (eventsRequested = eventsAccepted = N).
>> OK: no SD write failures.
```

Treat a `!! WARNING:` line as a failed run and investigate before flying again;
it usually means the firmware on the drone is not the build described here.

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
