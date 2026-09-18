#!/usr/bin/env python3
"""Fly a terrain test pattern under Lighthouse while logging flow deck data to the SD card, then
validate the log.

Needs the rik/terrain-logging firmware: ToF and flow are logged on estTOF/estFlow but not fused
(kalman.ignoreTof/ignoreFlow = 1), so the Lighthouse estimate is the ground truth. Copy
tools/usdlog/terrain/config.txt to the SD card.

Patterns (all absolute positions, yaw held at 0):
  line   Back and forth between --a and --b at --z, once per speed in --speeds.
         Put obstacles or a ramp between a and b.
  climb  Up and down between --z-low and --z-high above --a, once per speed in --vspeeds.
         Fly this over flat floor: it is the "I moved, not the ground" case.
  hover  Hover at --a, --z for --duration seconds. Put --a over a box edge for the edge case,
         or over flat floor for baseline drift.

Validation, in two steps:
  1. After landing, logging is stopped and the uSD counters are read over the radio: dropped
     events (usd.eventsRequested vs usd.eventsAccepted) and SD write errors (usd.writeError).
  2. The script asks you to move the SD card to a card reader, copies the newest log file into
     --out-dir and checks the file itself: CRC, event count against the counter, all event types
     present, and gaps in the 1 kHz IMU and 100 Hz ground truth streams.
Each run is appended to <out-dir>/runs.jsonl with its settings, results and --notes.

To fly several runs before taking the card out, pass --skip-fetch to each run (the counters are
still checked after every flight), then copy and check all of them at once with the fetch
subcommand. Re-check a single log file with the check subcommand.

Run from this directory, where the uv project has cflib2, tyro and numpy:
    cd tools/usdlog
    uv run fly_terrain_log.py fly --pattern line --a -1.0 0.0 --b 1.0 0.0 --z 0.8 --notes "boxes 20 and 30 cm"
    uv run fly_terrain_log.py check terrain_runs/20260918-1130_line.bin
"""

import asyncio
import glob
import json
import math
import os
import shutil
import struct
import time
from dataclasses import asdict, dataclass, field
from typing import Annotated, Literal, Optional, Union
from zlib import crc32

import numpy as np
import tyro

import cfusdlog
from cflib2 import Crazyflie, LinkContext

# The 1 kHz gyro and accelerometer events alone give 2000 events/s. A run with fewer than this
# per second of logging did not log properly, even if no events were dropped: the firmware reports
# a failed file open as zero events and no write error.
MIN_EVENTS_PER_S = 1500

# Every event type in terrain/config.txt, with the largest gap between two events that is not
# a fault [ms]. None means the event is not periodic (flow is only sent when the sensor sees
# motion), so its gaps are reported but not checked.
EXPECTED_EVENTS = {
    "fixedFrequency": 25.0,
    "estGyroscope": 5.0,
    "estAcceleration": 5.0,
    "estTOF": 100.0,
    "estFlow": None,
    "estBarometer": 100.0,
}

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "terrain", "config.txt")


@dataclass
class Fly:
    pattern: Literal["line", "climb", "hover"]
    """Flight pattern, see the module docstring"""
    uri: str = "radio://0/80/2M/E7E7E7E7E7"
    """Crazyflie URI"""
    a: tuple[float, float] = (0.0, 0.0)
    """Start point (x, y) [m]"""
    b: tuple[float, float] = (1.0, 0.0)
    """End point (x, y) for the line pattern [m]"""
    z: float = 0.8
    """Flight height for line and hover [m]"""
    speeds: list[float] = field(default_factory=lambda: [0.3, 0.6, 1.0])
    """Horizontal speeds for the line pattern, one out-and-back per speed [m/s]"""
    z_low: float = 0.4
    """Low height for the climb pattern [m]"""
    z_high: float = 1.4
    """High height for the climb pattern [m]"""
    vspeeds: list[float] = field(default_factory=lambda: [0.2, 0.5, 0.8])
    """Vertical speeds for the climb pattern, one up-and-down per speed [m/s]"""
    duration: float = 30.0
    """Hover time for the hover pattern [s]"""
    settle: float = 2.0
    """Hover time at each end point, so steps and speed changes are separated in the log [s]"""
    notes: str = ""
    """Free text stored in runs.jsonl, e.g. box heights and positions"""
    dry_run: bool = False
    """Do everything except arming and flying: log for --dry-run-time seconds, then validate"""
    dry_run_time: float = 15.0
    """Logging time for --dry-run [s]. Move the drone by hand meanwhile to get flow events"""
    skip_fetch: bool = False
    """Leave the SD card in the drone to fly several runs in a row, then fetch them all with 'fetch'"""
    out_dir: str = "terrain_runs"
    """Directory the log file is copied to, and where runs.jsonl is kept"""
    card_dir: Optional[str] = None
    """Mount point of the SD card in the card reader, found under /media/$USER if not given"""
    log_name: str = "terr"
    """Log file name prefix, as on line 3 of config.txt"""


@dataclass
class Check:
    log_file: tyro.conf.Positional[str]
    """Log file to check"""
    expected_events: Optional[int] = None
    """usd.eventsAccepted read after the flight, to compare with the number of events in the file"""
    config: str = DEFAULT_CONFIG
    """config.txt the log was recorded with, to check that every variable in it was logged"""


@dataclass
class Fetch:
    out_dir: str = "terrain_runs"
    """Directory with runs.jsonl, and where the log files are copied to"""
    card_dir: Optional[str] = None
    """Mount point of the SD card in the card reader, found under /media/$USER if not given"""
    log_name: str = "terr"
    """Log file name prefix, as on line 3 of config.txt"""
    config: Optional[str] = None
    """config.txt the logs were recorded with, the one on the SD card if not given"""


# ---------------------------------------------------------------------------------------------
# Log file validation


def read_config_variables(path: str) -> dict[str, list[str]]:
    """Variables per event in a uSD config.txt. The deck silently skips names it does not know."""
    events: dict[str, list[str]] = {}
    current = None
    with open(path) as f:
        for line in f.readlines()[4:]:  # version, buffer size, file name, enable on startup
            token = line.split("#")[0].strip()
            if not token or token[0].isdigit():  # empty, or fixedFrequency's frequency and mode
                continue
            if token.startswith("on:"):
                current = token[3:]
                events[current] = []
            elif current is not None:
                events[current].append(token)
    return events


def check_log_file(path: str, expected_events: Optional[int], config: str = DEFAULT_CONFIG) -> bool:
    ok = True
    print(f"Checking {path}")

    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 9 or data[0] != 0xBC:
        print("!! Not a uSD log file (bad magic byte or too short)")
        return False
    if crc32(data[:-4]) != struct.unpack("<I", data[-4:])[0]:
        print("!! CRC mismatch: the file is truncated or corrupt")
        ok = False
    else:
        print("OK: CRC")

    log = cfusdlog.decode(path)
    if log is None:
        print("!! Could not decode the file")
        return False

    total = sum(len(event["timestamp"]) for event in log.values())
    if expected_events is not None:
        if total != expected_events:
            print(f"!! File has {total} events, the drone accepted {expected_events}")
            ok = False
        else:
            print(f"OK: file has all {total} events the drone accepted")
    else:
        print(f"   {total} events (no expected count given)")

    print(f"   {'event':<16} {'count':>8} {'rate [Hz]':>10} {'max gap [ms]':>13}")
    for name, max_gap in EXPECTED_EVENTS.items():
        if name not in log:
            print(f"!! {name:<16} missing: check config.txt and that the deck is attached")
            ok = False
            continue
        t = log[name]["timestamp"]
        gaps = np.diff(t)
        duration_s = (t[-1] - t[0]) / 1000.0
        rate = (len(t) - 1) / duration_s if duration_s > 0 else 0.0
        largest = gaps.max() if len(gaps) else 0.0
        flag = "  "
        if np.any(gaps < 0):
            flag = "!!"
            ok = False
            print(f"!! {name}: timestamps go backwards")
        elif max_gap is not None and largest > max_gap:
            flag = "!!"
            ok = False
        print(f"{flag} {name:<16} {len(t):>8} {rate:>10.1f} {largest:>13.1f}")
        if flag == "!!" and max_gap is not None and largest > max_gap:
            n_bad = int(np.sum(gaps > max_gap))
            print(f"   {n_bad} gaps over {max_gap:.0f} ms, first at t = {t[1:][gaps > max_gap][0]:.0f} ms")

    missing = [f"{event}: {name}" for event, names in read_config_variables(config).items()
               if event in log for name in names if name not in log[event]]
    if missing:
        print(f"!! Variables in {os.path.basename(config)} but not in the log (wrong name, or not in this firmware):")
        for m in missing:
            print(f"     {m}")
        ok = False
    else:
        print(f"OK: every variable in {os.path.basename(config)} is in the log")

    if "estTOF" in log and "zranger2.status" in log["estTOF"]:
        status = log["estTOF"]["zranger2.status"]
        invalid = int(np.sum(status != 0))
        print(f"   ToF: {invalid} of {len(status)} ranges had a non-zero status (the driver sends them to the estimator anyway)")

    print("Log file OK" if ok else "!! Log file FAILED, do not use it for replay")
    return ok


def find_card_dir(card_dir: Optional[str], log_name: str) -> Optional[str]:
    if card_dir is not None:
        return card_dir
    user = os.environ.get("USER", "")
    candidates = [d for d in glob.glob(f"/media/{user}/*") + glob.glob(f"/run/media/{user}/*")
                  if glob.glob(os.path.join(d, "config.txt"))]
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        print(f"!! Several cards mounted, pass --card-dir: {candidates}")
    return None


def newest_log(card_dir: str, log_name: str) -> Optional[str]:
    # The Crazyflie has no clock, so file times are meaningless: the highest number is the newest
    files = [f for f in os.listdir(card_dir)
             if f.lower().startswith(log_name.lower()) and f[len(log_name):].isdigit()]
    if not files:
        return None
    return os.path.join(card_dir, max(files, key=lambda f: int(f[len(log_name):])))


def fetch_log(args: Fly, run_name: str) -> Optional[str]:
    answer = input("\nMove the SD card to the card reader and press Enter (s + Enter to skip): ")
    if answer.strip().lower() == "s":
        return None
    card_dir = find_card_dir(args.card_dir, args.log_name)
    while card_dir is None or newest_log(card_dir, args.log_name) is None:
        answer = input(f"No card with {args.log_name}NN files found, press Enter to retry (s + Enter to skip): ")
        if answer.strip().lower() == "s":
            return None
        card_dir = find_card_dir(args.card_dir, args.log_name)
    source = newest_log(card_dir, args.log_name)
    os.makedirs(args.out_dir, exist_ok=True)
    target = os.path.join(args.out_dir, f"{run_name}.bin")
    shutil.copyfile(source, target)
    print(f"Copied {source} to {target}")
    return target


def count_events(path: str) -> Optional[int]:
    """Events in a log file, or None if it cannot be decoded (empty or cut off by a failed run)"""
    try:
        log = cfusdlog.decode(path)
    except Exception:
        return None
    return None if log is None else sum(len(event["timestamp"]) for event in log.values())


def fetch_pending(args: Fetch) -> bool:
    """Copy and check the logs of all runs flown with --skip-fetch.

    The drone does not report which file it wrote, so files are matched to runs by event count:
    a complete file holds exactly the usd.eventsAccepted recorded for its run.
    """
    runs_path = os.path.join(args.out_dir, "runs.jsonl")
    with open(runs_path) as f:
        records = [json.loads(line) for line in f if line.strip()]
    pending = [r for r in records if r.get("log_file") is None and r.get("usd")]
    for r in [r for r in pending if r.get("counters_ok") is False]:
        print(f"   {r['run']}: skipped, its uSD counters already showed dropped events or a write error")
    pending = [r for r in pending if r.get("counters_ok") is not False]
    if not pending:
        print("No runs waiting for their log file")
        return True

    card_dir = find_card_dir(args.card_dir, args.log_name)
    if card_dir is None:
        print("!! No SD card found, pass --card-dir")
        return False
    config = args.config or os.path.join(card_dir, "config.txt")
    files = sorted((f for f in os.listdir(card_dir)
                    if f.lower().startswith(args.log_name.lower()) and f[len(args.log_name):].isdigit()),
                   key=lambda f: int(f[len(args.log_name):]))
    by_count: dict[int, list[str]] = {}
    for f in files:
        n = count_events(os.path.join(card_dir, f))
        if n is not None:
            by_count.setdefault(n, []).append(f)

    all_ok = True
    for r in pending:
        matches = by_count.get(r["usd"]["eventsAccepted"], [])
        if len(matches) != 1:
            print(f"!! {r['run']}: {len(matches)} files with {r['usd']['eventsAccepted']} events, "
                  f"check it by hand with the check subcommand")
            all_ok = False
            continue
        source = os.path.join(card_dir, matches[0])
        target = os.path.join(args.out_dir, f"{r['run']}.bin")
        shutil.copyfile(source, target)
        print(f"\n{r['run']}: copied {source} to {target}")
        r["log_file"] = target
        r["file_ok"] = check_log_file(target, r["usd"]["eventsAccepted"], config)
        all_ok &= r["file_ok"]

    with open(runs_path, "w") as f:
        f.writelines(json.dumps(r) + "\n" for r in records)
    return all_ok


# ---------------------------------------------------------------------------------------------
# Flight


async def wait_for(supervisor, predicate, what: str, timeout: float = 10.0) -> None:
    deadline = asyncio.get_event_loop().time() + timeout
    while True:
        state = await supervisor.read()
        if predicate(state):
            return
        if asyncio.get_event_loop().time() > deadline:
            raise RuntimeError(f"timed out waiting for {what}, supervisor states: {state.active_states()}")
        await asyncio.sleep(0.2)


async def check_firmware(cf) -> None:
    """Refuse to fly unless the firmware, decks and SD card are set up for terrain logging"""
    param = cf.param()
    names = set(param.names())

    for name in ("kalman.ignoreTof", "kalman.ignoreFlow"):
        if name not in names:
            raise RuntimeError(f"{name} not found: flash the rik/terrain-logging firmware")
        if await param.get(name) != 1:
            raise RuntimeError(f"{name} is 0: ToF/flow would be fused and the flight would not be at constant height")

    for deck in ("deck.bcFlow2", "deck.bcLighthouse4", "deck.bcUSD"):
        if deck not in names or await param.get(deck) != 1:
            raise RuntimeError(f"{deck} is not attached")

    if "zranger2.sigma" not in cf.log().names():
        raise RuntimeError("zranger2 log group not found: flash the rik/terrain-logging firmware")

    if not await param.get("usd.canLog"):
        raise RuntimeError("usd.canLog is 0: no SD card, or the config.txt is not valid")


async def read_usd_stats(cf) -> dict:
    """Read the uSD integrity counters. They are reset when logging starts and hold after it stops."""
    block = await cf.log().create_block()
    for name in ("usd.eventsRequested", "usd.eventsAccepted", "usd.writeError"):
        await block.add_variable(name)
    stream = await block.start(100)
    sample = await asyncio.wait_for(stream.next(), timeout=3.0)
    await stream.stop()
    return {
        "eventsRequested": sample.data["usd.eventsRequested"],
        "eventsAccepted": sample.data["usd.eventsAccepted"],
        "writeError": sample.data["usd.writeError"],
    }


def report_usd_stats(stats: dict) -> bool:
    if stats["eventsAccepted"] is None:
        print("!! No uSD counters: the connection was lost before they could be read")
        return False
    ok = True
    requested, accepted = stats["eventsRequested"], stats["eventsAccepted"]
    if requested != accepted:
        print(f"!! DROPPED EVENTS: {requested - accepted} of {requested} lost. The log has holes, do not use it.")
        print("   Increase the buffer size (line 2 of config.txt) or log fewer variables.")
        ok = False
    else:
        print(f"OK: no dropped events ({accepted} events)")
    minimum = MIN_EVENTS_PER_S * stats.get("logged_s", 0)
    if accepted < minimum:
        print(f"!! TOO FEW EVENTS: {accepted} in {stats['logged_s']:.0f} s of logging, expected at least {minimum:.0f}.")
        print("   Logging did not start or stopped early, most likely the file could not be opened.")
        print("   Power-cycle the drone so it mounts the SD card again.")
        ok = False
    if stats["writeError"] != 0:
        print(f"!! SD WRITE FAILURE: usd.writeError = {stats['writeError']} (FatFS FRESULT). The log is incomplete.")
        print("   Power-cycle the drone before the next run, or the next log file may fail to open.")
        ok = False
    else:
        print("OK: no SD write errors")
    return ok


async def fly_line(hlc, args: Fly) -> None:
    (ax, ay), (bx, by) = args.a, args.b
    distance = math.hypot(bx - ax, by - ay)
    for speed in args.speeds:
        leg_time = distance / speed
        for i, (x, y) in enumerate([(bx, by), (ax, ay)]):
            print(f"{speed:.2f} m/s, leg {i + 1}/2: to ({x:+.2f}, {y:+.2f}) in {leg_time:.1f} s")
            await hlc.go_to(x, y, args.z, 0.0, leg_time, False, False, None)
            await asyncio.sleep(leg_time + args.settle)


async def fly_climb(hlc, args: Fly) -> None:
    ax, ay = args.a
    for speed in args.vspeeds:
        leg_time = (args.z_high - args.z_low) / speed
        for z in (args.z_high, args.z_low):
            print(f"{speed:.2f} m/s: to z = {z:.2f} in {leg_time:.1f} s")
            await hlc.go_to(ax, ay, z, 0.0, leg_time, False, False, None)
            await asyncio.sleep(leg_time + args.settle)


async def fly_hover(hlc, args: Fly) -> None:
    print(f"Hovering for {args.duration:.0f} s")
    await asyncio.sleep(args.duration)


async def fly(args: Fly) -> tuple[bool, Optional[dict]]:
    """Fly the pattern with SD logging on. Returns whether the pattern completed and the uSD counters."""
    print(f"Connecting to {args.uri}...")
    ctx = LinkContext()
    cf = await Crazyflie.connect_from_uri(ctx, args.uri)
    print("Connected")

    param = cf.param()
    supervisor = cf.supervisor()
    hlc = cf.high_level_commander()

    try:
        await check_firmware(cf)
    except Exception:
        await cf.disconnect()
        raise

    logging_started = False
    logging_stopped = False
    logged_s = 0.0
    airborne = False
    completed = False
    stats = None
    try:
        print("Starting SD logging")
        await param.set("usd.logging", 1)
        logging_started = True
        logging_start = time.monotonic()
        await asyncio.sleep(1.0)

        # Never fly without a working log: after an SD error the file can fail to open, which the
        # firmware reports as zero events and no error
        early = await read_usd_stats(cf)
        if early["eventsAccepted"] < MIN_EVENTS_PER_S * 0.5 or early["writeError"] != 0:
            raise RuntimeError(f"SD logging is not working ({early['eventsAccepted']} events after 1 s, "
                               f"writeError {early['writeError']}). Power-cycle the drone and try again. Not arming.")
        print(f"SD logging OK ({early['eventsAccepted']} events in the first second)")

        if args.dry_run:
            print(f"Dry run: logging for {args.dry_run_time:.0f} s without arming")
            await asyncio.sleep(args.dry_run_time)
        else:
            await wait_for(supervisor, lambda s: s.can_be_armed, "can_be_armed")
            print("Arming")
            await supervisor.send_arming_request(True)
            await wait_for(supervisor, lambda s: s.is_armed, "is_armed")
            await wait_for(supervisor, lambda s: s.can_fly, "can_fly")

            height = args.z_low if args.pattern == "climb" else args.z
            print(f"Taking off to {height:.2f} m")
            await hlc.take_off(height, None, 3.0, None)
            airborne = True
            await asyncio.sleep(3.0)

            ax, ay = args.a
            print(f"To start point ({ax:+.2f}, {ay:+.2f})")
            await hlc.go_to(ax, ay, height, 0.0, 3.0, False, False, None)
            await asyncio.sleep(3.0 + args.settle)

            await {"line": fly_line, "climb": fly_climb, "hover": fly_hover}[args.pattern](hlc, args)

        # Stop before landing: SD write errors have been seen during landing and disarm, and the
        # terrain data does not need them
        print("Stopping SD logging")
        await param.set("usd.logging", 0)
        logged_s = time.monotonic() - logging_start
        logging_stopped = True
        completed = True

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("Interrupted")
    finally:
        if airborne:
            print("Landing")
            try:
                await hlc.land(0.0, None, 3.0, None)
                await asyncio.sleep(3.5)
            finally:
                await hlc.stop(None)
        print("Disarming")
        await supervisor.send_arming_request(False)
        if logging_started:
            if not logging_stopped:
                await asyncio.sleep(0.5)
                print("Stopping SD logging")
                await param.set("usd.logging", 0)
                logged_s = time.monotonic() - logging_start
            await asyncio.sleep(1.0)
            try:
                stats = await read_usd_stats(cf)
                stats["logged_s"] = round(logged_s, 1)
            except Exception as e:
                print(f"!! Could not read the uSD counters ({type(e).__name__}: {e}). Treat this run's log as bad.")
                stats = {"eventsRequested": None, "eventsAccepted": None, "writeError": None, "logged_s": round(logged_s, 1)}
        try:
            await cf.disconnect()
        except Exception:
            pass

    return completed, stats


def run_fly(args: Fly) -> None:
    run_name = f"{time.strftime('%Y%m%d-%H%M%S')}_{args.pattern}"
    completed, stats = asyncio.run(fly(args))
    if stats is None:
        return

    print("\nuSD counters:")
    counters_ok = report_usd_stats(stats)

    log_path = None if args.skip_fetch else fetch_log(args, run_name)
    # Check against the config.txt the drone used, the one on the card
    if log_path:
        card_dir = find_card_dir(args.card_dir, args.log_name)
        card_config = os.path.join(card_dir, "config.txt") if card_dir else DEFAULT_CONFIG
    file_ok = check_log_file(log_path, stats["eventsAccepted"], card_config) if log_path else None

    record = {
        "run": run_name,
        "completed": completed,
        "counters_ok": counters_ok,
        "file_ok": file_ok,
        "log_file": log_path,
        "usd": stats,
        **{k: v for k, v in asdict(args).items() if k not in ("out_dir", "card_dir", "log_name")},
    }
    os.makedirs(args.out_dir, exist_ok=True)
    with open(os.path.join(args.out_dir, "runs.jsonl"), "a") as f:
        f.write(json.dumps(record) + "\n")
    print(f"Run recorded as {run_name} in {args.out_dir}/runs.jsonl")
    if log_path is None:
        print("Log file not checked yet. After the last run, with the card in the reader: uv run fly_terrain_log.py fetch")


def main() -> None:
    args = tyro.cli(Union[
        Annotated[Fly, tyro.conf.subcommand("fly")],
        Annotated[Check, tyro.conf.subcommand("check")],
        Annotated[Fetch, tyro.conf.subcommand("fetch")],
    ])
    if isinstance(args, Fly):
        run_fly(args)
    elif isinstance(args, Fetch):
        raise SystemExit(0 if fetch_pending(args) else 1)
    else:
        raise SystemExit(0 if check_log_file(args.log_file, args.expected_events, args.config) else 1)


if __name__ == "__main__":
    main()
