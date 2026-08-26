# Link-free autonomous flight for TDoA capture

A radio link degrades Loco performance, so a clean TDoA candidate capture has to
fly with the radio off — which means nothing can command the drone during the
run. This app flies a predefined sequence on its own: trigger it over a brief
connection, disconnect, and it arms, flies, lands and disarms unattended.

It is an **optional** companion to `tools/usdlog/`. The capture tooling there
does not depend on it, and everything it records is still produced by firmware
built without it.

## What it does not do

**It does not start the log.** `usd.logging` and `tdoaEngine.logCand` stay under
script control, so `tools/usdlog/read_usd_log.sh` and `config.txt` keep working
against any firmware build. The app only *stops* logging at touchdown, and only
because an unclosed log file is 0 bytes: without that, a battery going flat
between landing and your next connection takes the whole capture with it. Set
`tdoaFlight.stopLog = 0` to turn even that off.

It does refuse to fly when `usd.canLog` is 0, which is the only signal that the
card mounted *and* `config.txt` parsed. Flying a capture that logs nothing costs
a battery and a repack. Set `tdoaFlight.reqLog = 0` to fly without the deck.

## Why the app has to do the arming

`PREFLIGHT_TIMEOUT_MS` is 30000 (`src/platform/interface/platform_defaults.h`)
and `supervisorIsPreflightTimeout()` measures it from the arming tick. An armed
drone that has not taken off within 30 s leaves `supervisorStateReadyToFly`, and
`supervisor.c` disarms it on the way out. So arming over the link and then
disconnecting gives you a 30 s window to get the radio down and be airborne —
too tight to rely on. The app arms immediately before takeoff instead.

## Build

From this directory, two commands:

```
cd examples/app_tdoa_flight
make cf21bl_defconfig        # or cf2_defconfig
make -j$(nproc)              # -> build/cf21bl.bin
```

`make <platform>_defconfig` configures and `make` builds; the split is there
because the first step is what you repeat when you change a fragment. It applies
the platform defconfig, merges `../../tools/usdlog/tdoa3_lh_groundtruth.conf`
and `app-config` on top, and runs `olddefconfig` to resolve the result. That
last pass is not optional — `merge_config.sh -m` skips it by design, and without
it the build stops and interviews you about the symbols the capture fragment
unlocks.

Output goes to `./build`, so this coexists with a normal build in the repo root
rather than fighting over `build/.config`.

The `Makefile` here does **not** include `tools/make/oot.mk`, whose `all` target
runs `oldconfig` rather than `olddefconfig` and would put that interview back.
It calls the root build directly with `OOT=$(CURDIR)`, which works because the
root Makefile has `objs-y += $(OOT)`.

All the `cfcli` and `tools/usdlog/` commands below are run **from the repo
root**, not from here.

## Flying a run

```
export CF_URI=radio://0/100/2M/F00D2BEFED

# 1. Preflight, as in tools/usdlog/README_tdoa_candidates.md
cfcli -u "$CF_URI" --csv param get \
    deck.bcLoco,deck.bcLighthouse4,deck.bcUSD,usd.canLog

# 2. Pick a sequence and arm candidate logging
cfcli -u "$CF_URI" param set tdoaEngine.logCand=1
cfcli -u "$CF_URI" param set tdoaFlight.seq=2,tdoaFlight.delay=15

# 3. Trigger, then start logging LAST and send nothing after it
cfcli -u "$CF_URI" param set tdoaFlight.start=1
cfcli -u "$CF_URI" param set usd.logging=1

# 4. After it lands, verify the capture over a link
tools/usdlog/read_usd_log.sh "$CF_URI" --check-only
```

`usd.logging` goes last on purpose. Once logging is running the uSD write task
is resumed on every event -- including `estAcceleration` and `estGyroscope`,
which fire per IMU sample at 1 kHz -- and a CRTP packet arriving into that load
can overflow `crtpPacketDelivery` and assert at `radiolink.c:171`, taking the
drone down hard enough to need a power cycle. Sending it last means nothing
arrives during the loaded window. `tdoaEngine.logCand` is safe earlier, since
event writes return immediately while `enableLogging` is 0.

The countdown is started by `start=1`, so `delay` has to cover the one remaining
command plus getting clear. 15 s is comfortable.

`tdoaFlight.start` is an edge trigger: the app clears it back to 0 as soon as it
accepts, so reading it back tells you nothing. Watch `tdoaFlight.state` instead.

Flying a second sequence needs no link at all if you are still within battery:
set `start` again and the uSD deck opens a new numbered file (`log00`, `log01`,
…), because `usddeck.c` walks `f_stat` for a free name on every logging start.
One sequence per file is the point — a fly-away is only attributable if you know
which motion produced it.

## Sequences

`tdoaFlight.seq` indexes the table in `src/flight_sequences.h`:

| seq | name | what it is for |
| --- | --- | --- |
| 0 | `hover` | Static reference. No motion, no antenna shadowing — replay divergence here is close to a floor. |
| 1 | `box` | Two laps of a square with corner pauses. All transients, four distinct anchor geometries. |
| 2 | `spiral` | Chained 90-degree spiral segments, expanding then contracting while climbing. Continuous motion, no stops. |
| 3 | `vertical` | Height sweep at two horizontal positions. z is TDoA's weak axis, so this is where errors show first. |
| 4 | random walk | Seeded pseudo-random waypoints. Same `tdoaFlight.seed` flies the same path, so an interesting run can be repeated exactly. |

Sequence coordinates are **normalised**: x and y are fractions of
`tdoaFlight.workXY` and z is a fraction between `tdoaFlight.zLo` and
`tdoaFlight.zHi`. Re-measuring the usable volume is three params, and every
sequence rescales itself.

Adding one means adding an array and a table entry in `flight_sequences.h`.
Appending is safe; **reordering is not** — the sequence index is recorded in the
log, so it retroactively renames every capture you already have.

## Envelope

| param | default | meaning |
| --- | --- | --- |
| `tdoaFlight.fenceXY` | 2.5 | Geofence half-width in x and y (m). Breach lands the drone. |
| `tdoaFlight.fenceZ` | 2.5 | Geofence ceiling (m). |
| `tdoaFlight.workXY` | 2.0 | Half-width of the working box (m); sequence +-1.0 maps here. |
| `tdoaFlight.zLo` | 0.5 | Height that sequence z = 0.0 maps to (m). |
| `tdoaFlight.zHi` | 2.0 | Height that sequence z = 1.0 maps to (m). |
| `tdoaFlight.lockThr` | 0.05 | Largest peak-to-peak movement allowed in any Kalman position variance over 2 s before takeoff. |

Keep `workXY` inside `fenceXY` with margin: the geofence is measured on the
*estimate*, which is exactly what is under suspicion, so it is a backstop and
not a boundary to fly against.

These are not persistent, so they need setting after every boot, in the same
connection as the trigger.

## Safety

There is no radio during the run, so every abort has to be onboard:

| condition | response |
| --- | --- |
| Estimator variance not converged before takeoff | refuse to arm |
| `usd.canLog == 0` (and `reqLog`) | refuse to arm |
| Geofence breach | land |
| `supervisorIsTumbled()` | motors off immediately |
| `pmIsBatteryLow()` | land |
| High level command rejected | land |
| Step over its duration + 5 s | land |
| Whole run over 7 minutes | land |

Takeoff is gated on the Kalman position **variances** having stopped moving, not
on the position having stopped moving: a diverged filter on a stationary drone
also has a still position, but its variance is still shrinking.

`tdoaFlight.lockThr` sets how still is still enough. The default of 0.05 is
sized for TDoA, which is far noisier than the 0.001 the Bitcraze demos use
against Lighthouse and Flow — a resting drone in this rig measured ~0.0125
peak-to-peak on `kalman.varPX` over 2 s, so a threshold near 0.001 never passes.

If a run sits at `state = 2` and never arms, that is this gate. Watch
`tdoaFlight.lockSpr`, which is the live value being compared against the
threshold, and set `lockThr` above what your system actually does:

```
cfcli -u "$CF_URI" --csv log print tdoaFlight.state,tdoaFlight.lockSpr
```

Raising it far enough will always let the drone arm, which is the point and also
the hazard: this gate is what stops a takeoff on a diverged estimate. Before
raising it, check that the estimate is right at all — with Lighthouse in
ground-truth mode you can compare directly:

```
cfcli -u "$CF_URI" --csv log print stateEstimate.x,stateEstimate.y,stateEstimate.z,lighthouse.x,lighthouse.y,lighthouse.z
```

Those should agree to a few centimetres on a resting drone. If they do not, the
anchor positions stored on the drone (`cfcli loco display`) probably do not
match the room, and no threshold will make flying safe.

## Reading the outcome

The app fires an event, `tdoaFlightStep`, on every state and step change,
carrying `seq`, `step`, `state` and `abortR`. One line in `config.txt` and every
capture says which sequence it was and where each step began:

```
on:tdoaFlightStep
```

An event rather than log variables in the 50 Hz block, deliberately: these
change a few dozen times in a whole flight, so sampling them periodically would
spend ring buffer bandwidth — the resource whose overrun silently puts holes in
the capture — on repeating a constant. The event is also strictly more
informative, since it carries the exact timestamp of each transition and lets
the log be segmented by flight step; periodic sampling only resolves that to
within a sample period.

Costs nothing in compatibility: `usddeck.c` prints `Unknown event` and skips
names it does not have, leaving `usd.canLog` at 1, so the same `config.txt`
still works on firmware built without this app.

The same four values are also plain log variables in the `tdoaFlight` group, for
watching a run live over a link with `cfcli log print`. Those cost nothing
unless a `config.txt` asks for them.

`state`: 0 idle, 1 countdown, 2 waiting for estimator lock, 3 arming,
4 taking off, 5 running, 6 landing, 7 done, 8 aborted.

`abortR`: 0 none, 1 no estimator lock, 2 cannot arm, 3 geofence, 4 tumbled,
5 battery low, 6 command rejected, 7 step timeout, 8 flight timeout, 9 logging
not ready, 10 bad sequence, 11 abort requested, 12 takeoff failed.

A run that ended cleanly finishes at `state = 7`, `abortR = 0`.

## Spiral geometry

`crtpCommanderHighLevelSpiral()` never sets the planner's `clockwise` field, so
it is always counter-clockwise. In forward mode the centre comes from the
current pose — `xCentre = x - r0*sin(yaw)`, `yCentre = y + r0*cos(yaw)` — so at
yaw 0 the centre is `r0` to the **left**. To circle the origin at radius `r`,
be at `(0, -r)` with yaw 0 first, which is what `seqSpiral` does.

`phi` saturates at +-2*pi and the API docstring describes a spline approximation
"for <= 90-degree segments", so full circles are chained 90-degree steps. Chained
segments only stay concentric if each `r0` equals the previous `rf`.
