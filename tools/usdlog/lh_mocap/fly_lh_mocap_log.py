#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "cflib2==0.1.0",
#     "qtm-rt==3.1.0",
#     "tyro",
#     "numpy",
# ]
# ///
"""Fly Lighthouse test patterns while logging to the SD card and recording Qualisys motion capture.

Needs the firmware of this branch (uSD counters, kalmanReset event), the Lighthouse, uSD and active
marker decks, and config.txt from this directory on the SD card. QTM must be running (preview or
capture) with a 6DOF rigid body for the drone.

One run is one flight pattern and one SD log file:
  1. Records the motion capture 6DOF stream to <out-dir>/<run>_mocap.csv, with QTM's timestamps.
  2. Puts the active markers in solid mode, starts SD logging and checks that events arrive.
  3. Sync pulses: switches the markers off and on a few times. Motion capture loses the drone during
     each pulse and the SD log has an activeMarkerModeChanged event at each switch.
  4. Resets the Kalman estimator (kalmanReset event in the log) and waits at rest.
  5. Takes off, flies the pattern, flies back to the start point at --sync-height and sends sync pulses
     again while hovering.
  6. Stops SD logging before landing, lands, and checks the uSD counters over the radio
     (dropped events, write errors).
Each run is appended to <out-dir>/runs.jsonl. Leave the SD card in the drone for a whole session,
then copy and check all log files at once with the fetch subcommand.

The Lighthouse geometry and calibration are not in the log. The first run of a session saves them to
<out-dir>/lh_config.yaml with cfcli.

Every setpoint of a pattern is checked against the flight area (--x-min ... --z-max) before connecting.

Adding a pattern: subclass Pattern with its settings as dataclass fields, implement setpoints() and
fly(), and add it to PATTERNS.

The Crazyflie URI, the QTM computer and the flight area of the local setup are read from
local_settings.json in this directory, which is not committed (copy local_settings.example.json).
The command line options override them.

Run from this directory with uv, which installs the dependencies from the header:
    uv run fly_lh_mocap_log.py hover --dry-run
    uv run fly_lh_mocap_log.py hover --heights 0.5 1.0 1.5
    uv run fly_lh_mocap_log.py figure8 --time-scales 1.0 0.7 0.5 --notes "first fast one"
    uv run fly_lh_mocap_log.py spin --peak-rates-deg 90 180 360
    uv run fly_lh_mocap_log.py wrong-heading --heading-deg 180
    uv run fly_lh_mocap_log.py random --seed 1 --points 20 --max-speed 2.0
    uv run fly_lh_mocap_log.py fetch
    uv run fly_lh_mocap_log.py check lh_mocap_runs/20261022-153000_hover.bin
"""

import asyncio
import csv
import glob
import json
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from typing import Annotated, ClassVar, Optional, Union
from zlib import crc32

import numpy as np
import qtm_rt
import tyro
from cflib2 import Crazyflie, LinkContext
from cflib2.memory import Poly, Poly4D

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import cfusdlog  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "config.txt")


def load_local_settings() -> dict:
    """Settings for this setup that are not committed, such as the URI and the QTM computer"""
    path = os.path.join(HERE, "local_settings.json")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        settings = json.load(f)
    unknown = set(settings) - {"uri", "qtm_host", "x_min", "x_max", "y_min", "y_max", "z_min", "z_max"}
    if unknown:
        print(f"!! Unknown keys in {path}, ignored: {sorted(unknown)}")
    return settings


LOCAL_SETTINGS = load_local_settings()

# The 1 kHz gyro and accelerometer events alone give 2000 events/s. A run with fewer than this
# per second of logging did not log properly, even if no events were dropped: the firmware reports
# a failed file open as zero events and no write error.
MIN_EVENTS_PER_S = 1500

# Periodic event types in config.txt, with the largest gap between two events that is not a fault [ms]
EXPECTED_PERIODIC_EVENTS = {
    "fixedFrequency": 25.0,
    "estGyroscope": 5.0,
    "estAcceleration": 5.0,
}
# Event types that must be in the log, but are not periodic: the sweeps stop when the deck is
# occluded, and the yaw error comes with the sweeps
EXPECTED_OTHER_EVENTS = ("estSweepAngle", "estYawError", "activeMarkerModeChanged", "kalmanReset")

# Active marker deck modes (activeMarkerDeck.c)
MARKER_MODE_OFF = 0
MARKER_MODE_SOLID = 1  # PWM: always on

TRAJECTORY_ID = 1

# Figure-8 from the cflib2 trajectory example (examples/trajectory.py). Each row: duration [s], then
# 8 polynomial coefficients each for x, y, z and yaw. Starts and ends at (0, 0, 0).
FIGURE8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa: E501
]


# ---------------------------------------------------------------------------------------------
# Flight area and patterns


@dataclass
class FlyOptions:
    """Settings shared by all patterns"""

    uri: str = LOCAL_SETTINGS.get("uri", "radio://0/80/2M/E7E7E7E7E7")
    """Crazyflie URI, default from local_settings.json"""
    x: float = 0.0
    """Start point x, where each pattern starts after take-off [m]"""
    y: float = 0.0
    """Start point y [m]"""
    x_min: float = LOCAL_SETTINGS.get("x_min", -1.5)
    """Flight area: every setpoint must lie inside it, default from local_settings.json [m]"""
    x_max: float = LOCAL_SETTINGS.get("x_max", 1.5)
    y_min: float = LOCAL_SETTINGS.get("y_min", -1.5)
    y_max: float = LOCAL_SETTINGS.get("y_max", 1.5)
    z_min: float = LOCAL_SETTINGS.get("z_min", 0.3)
    z_max: float = LOCAL_SETTINGS.get("z_max", 2.0)
    max_speed: Optional[float] = None
    """Raise the PID position controller's speed limit (posCtlPid.xVelMax/yVelMax, default 1.0) for
    this run [m/s]. With position setpoints it caps the speed at 1.1 times this value. Restored after
    the run"""
    max_tilt: Optional[float] = None
    """Raise the PID velocity controller's roll and pitch limit (posCtlPid.rLimit/pLimit, default 20)
    for this run [deg]. Restored after the run"""
    sync_height: float = 1.0
    """Height to fly to before the end sync pulses, where motion capture is known to see the drone [m]"""
    move_speed: float = 0.5
    """Speed for moves between the parts of a pattern [m/s]"""
    rest_s: float = 3.0
    """Time at rest after the estimator reset, before arming [s]"""
    settle_s: float = 2.0
    """Hover time between the parts of a pattern, so they are separated in the log [s]"""
    qtm_host: str = LOCAL_SETTINGS.get("qtm_host", "")
    """Computer running QTM, default from local_settings.json"""
    qtm_body: str = "cf"
    """Name of the drone's 6DOF rigid body in QTM"""
    marker_brightness: int = 100
    """Brightness of the front, left and right active markers (back is off), 0..255"""
    sync_pulses: int = 3
    """Number of marker off/on pulses at the start and at the end of a run"""
    sync_pulse_s: float = 0.3
    """Duration of each off and each on phase of a sync pulse [s]"""
    notes: str = ""
    """Free text stored in runs.jsonl"""
    dry_run: bool = False
    """Do everything except arming and flying: log at rest for --dry-run-time seconds instead"""
    dry_run_time: float = 10.0
    """Logging time at rest for --dry-run [s]. Move the drone by hand meanwhile if useful"""
    out_dir: str = "lh_mocap_runs"
    """Directory for runs.jsonl, motion capture files, console logs and fetched log files"""


Point = tuple[float, float, float]


class Pattern(FlyOptions):
    """A flight pattern. Subclasses are dataclasses with the pattern's settings as fields."""

    name: ClassVar[str]

    def first_height(self) -> float:
        raise NotImplementedError

    def setpoints(self) -> list[Point]:
        """Every position the pattern commands, absolute, for the flight area check"""
        raise NotImplementedError

    async def prepare(self) -> dict:
        """Called before logging starts. Returns extra information for runs.jsonl."""
        return {}

    async def fly(self, f: "Flight") -> None:
        """Fly the pattern. Starts hovering at (x, y, first_height()), must end hovering."""
        raise NotImplementedError


@dataclass
class Hover(Pattern):
    """Hover at each height in turn"""

    name: ClassVar[str] = "hover"
    heights: list[float] = field(default_factory=lambda: [0.5, 1.0, 1.5])
    """Hover heights, in this order [m]"""
    hover_s: float = 5.0
    """Hover time at each height [s]"""

    def first_height(self) -> float:
        return self.heights[0]

    def setpoints(self) -> list[Point]:
        return [(self.x, self.y, z) for z in self.heights]

    async def fly(self, f: "Flight") -> None:
        for z in self.heights:
            await f.go_to(self.x, self.y, z)
            print(f"Hovering at {z:.2f} m for {self.hover_s:.0f} s")
            await f.hold(self.hover_s)


def figure8_trajectory() -> list[Poly4D]:
    return [Poly4D(row[0], Poly(row[1:9]), Poly(row[9:17]), Poly(row[17:25]), Poly(row[25:33]))
            for row in FIGURE8]


def figure8_points(n_per_piece: int = 50) -> np.ndarray:
    """Positions along the figure-8, relative to its start, sampled for the flight area check"""
    points = []
    for row in FIGURE8:
        t = np.linspace(0.0, row[0], n_per_piece)
        points.append(np.stack([np.polyval(row[1 + 8 * i:9 + 8 * i][::-1], t) for i in range(3)], axis=1))
    return np.concatenate(points)


@dataclass
class Figure8(Pattern):
    """The cflib figure-8 trajectory at several speeds and heights. Yaw is held at 0."""

    name: ClassVar[str] = "figure8"
    heights: list[float] = field(default_factory=lambda: [1.0])
    """Heights to fly the figure-8 at, in this order [m]"""
    time_scales: list[float] = field(default_factory=lambda: [1.0, 0.7, 0.5])
    """Time scales, flown in this order at each height. 1.0 is the original speed, 0.5 twice as fast"""

    def first_height(self) -> float:
        return self.heights[0]

    def setpoints(self) -> list[Point]:
        rel = figure8_points()
        return [(self.x + px, self.y + py, z + pz) for z in self.heights for px, py, pz in rel]

    async def fly(self, f: "Flight") -> None:
        trajectory = figure8_trajectory()
        duration = sum(row[0] for row in FIGURE8)
        await f.cf.memory().write_trajectory(trajectory)
        await f.hlc.define_trajectory(TRAJECTORY_ID, 0, len(trajectory), 0)
        for z in self.heights:
            await f.go_to(self.x, self.y, z)
            await f.hold(self.settle_s)
            for scale in self.time_scales:
                print(f"Figure-8 at {z:.2f} m, time scale {scale:.2f} ({duration * scale:.1f} s)")
                await f.trajectory(TRAJECTORY_ID, scale, duration * scale)
                await f.hold(self.settle_s)


@dataclass
class Spin(Pattern):
    """Hover and turn in yaw in quarter turns, at increasing rates"""

    name: ClassVar[str] = "spin"
    height: float = 1.0
    """Hover height [m]"""
    peak_rates_deg: list[float] = field(default_factory=lambda: [90.0, 180.0, 360.0, 720.0])
    """Peak yaw rates, in this order [deg/s]"""
    turns: int = 2
    """Full turns per rate; the direction alternates between rates"""

    def first_height(self) -> float:
        return self.height

    def setpoints(self) -> list[Point]:
        return [(self.x, self.y, self.height)]

    async def fly(self, f: "Flight") -> None:
        # go_to plans yaw along the shortest angle, so a turn is flown as quarter turns. Each
        # quarter turn is a smooth rest-to-rest polynomial with a peak rate 35/16 times its mean rate.
        quarter = math.pi / 2
        for i, rate in enumerate(self.peak_rates_deg):
            direction = 1.0 if i % 2 == 0 else -1.0
            duration = (35.0 / 16.0) * quarter / math.radians(rate)
            print(f"Spinning {self.turns} turns at {rate:.0f} deg/s peak ({duration:.2f} s per quarter turn)")
            for _ in range(4 * self.turns):
                await f.turn(direction * quarter, duration)
            await f.hold(self.settle_s)


@dataclass
class WrongHeading(Pattern):
    """Start at a heading the estimator does not expect, then take off and hover"""

    name: ClassVar[str] = "wrong-heading"
    heading_deg: float = 180.0
    """Heading to put the drone down at, counter-clockwise from the +x axis [deg]"""
    height: float = 1.0
    """Hover height [m]"""
    hover_s: float = 5.0
    """Hover time [s]"""
    rest_s: float = 5.0
    """Time at rest after the estimator reset, before arming [s]"""

    def first_height(self) -> float:
        return self.height

    def setpoints(self) -> list[Point]:
        return [(self.x, self.y, self.height)]

    async def prepare(self) -> dict:
        await ask(f"\nPut the drone down at a heading of about {self.heading_deg:.0f} deg (counter-clockwise "
                  f"from +x), then press Enter: ")
        return {"heading_deg": self.heading_deg}

    async def fly(self, f: "Flight") -> None:
        print(f"Hovering for {self.hover_s:.0f} s")
        await f.hold(self.hover_s)


@dataclass
class RandomWaypoints(Pattern):
    """Fly through random waypoints with random yaws, at random speeds. The same seed gives the same flight."""

    name: ClassVar[str] = "random"
    seed: int = 0
    """Seed of the random waypoints, durations and yaws"""
    points: int = 20
    """Number of waypoints"""
    margin: float = 0.3
    """Distance of the waypoints from the x and y edges of the flight area [m]"""
    height: float = 1.0
    """Height of the start point, where the drone goes after take-off [m]"""
    speed_fraction: tuple[float, float] = (0.25, 0.75)
    """The mean speed of each move is a uniform random fraction in this range of the speed limit
    (posCtlPid.xVelMax, see --max-speed), or the mean yaw rate of --max-yaw-rate, whichever is reached
    first. A go_to move peaks at about twice its mean, so the faster moves are capped by the controller"""
    max_yaw_rate: float = 240.0
    """Yaw rate taken as the limit, the highest measured in a spin [deg/s]"""

    def first_height(self) -> float:
        return self.height

    def waypoints(self) -> list[dict]:
        # random.Random gives the same sequence for the same seed on every Python version
        rng = random.Random(self.seed)
        result = []
        for _ in range(self.points):
            result.append({
                "x": round(rng.uniform(self.x_min + self.margin, self.x_max - self.margin), 3),
                "y": round(rng.uniform(self.y_min + self.margin, self.y_max - self.margin), 3),
                "z": round(rng.uniform(self.z_min, self.z_max), 3),
                "yaw_deg": round(rng.uniform(-180.0, 180.0), 1),
                "fraction": round(rng.uniform(*self.speed_fraction), 3),
            })
        return result

    def setpoints(self) -> list[Point]:
        return [(w["x"], w["y"], w["z"]) for w in self.waypoints()]

    async def prepare(self) -> dict:
        await ask("\nPut the drone down anywhere in the flight area, at any heading, then press Enter: ")
        return {"waypoints": self.waypoints()}

    async def fly(self, f: "Flight") -> None:
        max_speed = await f.cf.param().get("posCtlPid.xVelMax")
        yaw = (await read_log_once(f.cf, ["stateEstimate.yaw"]))["stateEstimate.yaw"]
        for i, w in enumerate(self.waypoints()):
            distance = math.dist(f.position, (w["x"], w["y"], w["z"]))
            turn = abs((w["yaw_deg"] - yaw + 180.0) % 360.0 - 180.0)
            fastest = max(distance / max_speed, turn / self.max_yaw_rate)
            duration = max(fastest / w["fraction"], 0.5)
            print(f"{i + 1}/{self.points}: to ({w['x']:+.2f}, {w['y']:+.2f}, {w['z']:.2f}), yaw {w['yaw_deg']:+.0f} deg "
                  f"in {duration:.1f} s")
            await f.go_to_absolute(w["x"], w["y"], w["z"], math.radians(w["yaw_deg"]), duration)
            yaw = w["yaw_deg"]


PATTERNS = (Hover, Figure8, Spin, WrongHeading, RandomWaypoints)


async def ask(prompt: str) -> str:
    """input() in a thread, so the motion capture stream keeps being read while waiting"""
    return await asyncio.to_thread(input, prompt)


def check_area(p: Pattern) -> None:
    """Raise if a setpoint of the pattern, or the start point, is outside the flight area"""
    start = (p.x, p.y, p.first_height())
    sync = (p.x, p.y, p.sync_height)
    for (x, y, z) in [start, sync] + p.setpoints():
        if not (p.x_min <= x <= p.x_max and p.y_min <= y <= p.y_max and p.z_min <= z <= p.z_max):
            raise ValueError(f"setpoint ({x:+.2f}, {y:+.2f}, {z:.2f}) is outside the flight area "
                             f"x [{p.x_min}, {p.x_max}], y [{p.y_min}, {p.y_max}], z [{p.z_min}, {p.z_max}]")


class Flight:
    """High-level commander moves with the flight area check. Positions are absolute; the moves are
    sent relative to the last setpoint so the yaw is left alone."""

    def __init__(self, cf, options: Pattern, position: Point):
        self.cf = cf
        self.hlc = cf.high_level_commander()
        self.o = options
        self.position = position

    def _check(self, x: float, y: float, z: float) -> None:
        o = self.o
        if not (o.x_min <= x <= o.x_max and o.y_min <= y <= o.y_max and o.z_min <= z <= o.z_max):
            raise ValueError(f"setpoint ({x:+.2f}, {y:+.2f}, {z:.2f}) is outside the flight area")

    async def go_to(self, x: float, y: float, z: float, duration: Optional[float] = None) -> None:
        self._check(x, y, z)
        px, py, pz = self.position
        distance = math.dist((x, y, z), self.position)
        if duration is None:
            duration = max(1.0, distance / self.o.move_speed)
        if distance > 0.01:
            await self.hlc.go_to(x - px, y - py, z - pz, 0.0, duration, True, False, None)
            await asyncio.sleep(duration)
        self.position = (x, y, z)

    async def go_to_absolute(self, x: float, y: float, z: float, yaw: float, duration: float) -> None:
        """Move to an absolute position and yaw [rad]"""
        self._check(x, y, z)
        await self.hlc.go_to(x, y, z, yaw, duration, False, False, None)
        await asyncio.sleep(duration)
        self.position = (x, y, z)

    async def turn(self, angle: float, duration: float) -> None:
        await self.hlc.go_to(0.0, 0.0, 0.0, angle, duration, True, False, None)
        await asyncio.sleep(duration)

    async def trajectory(self, trajectory_id: int, time_scale: float, duration: float) -> None:
        await self.hlc.start_trajectory(trajectory_id, time_scale, True, False, False, None)
        await asyncio.sleep(duration)

    async def hold(self, seconds: float) -> None:
        await asyncio.sleep(seconds)


# ---------------------------------------------------------------------------------------------
# Motion capture


class MocapRecorder:
    """Streams the 6DOF pose of one rigid body from QTM to a CSV file.

    Columns: QTM timestamp [us], QTM frame number, position x, y, z [mm], rotation matrix as QTM
    sends it (9 values), PC time at arrival [s]. Untracked frames have NaN pose values.
    """

    HEADER = ["qtm_timestamp_us", "frame", "x_mm", "y_mm", "z_mm"] + [f"r{i}" for i in range(9)] + ["pc_time_s"]

    def __init__(self, host: str, body: str, path: str):
        self.host = host
        self.body = body
        self.path = path
        self.connection = None
        self.file = None
        self.writer = None
        self.body_index = None
        self.frames = 0
        self.tracked = 0
        self.timestamps: list[int] = []
        self.is_tracked: list[bool] = []
        self.markers_on_frame = 0

    async def start(self) -> None:
        if not self.host:
            raise RuntimeError("no QTM computer given: set qtm_host in local_settings.json or pass --qtm-host")
        print(f"Connecting to QTM on {self.host}")
        self.connection = await qtm_rt.connect(self.host, version="1.25")
        if self.connection is None:
            raise RuntimeError(f"could not connect to QTM on {self.host}")
        xml = ET.fromstring(await self.connection.get_parameters(parameters=["6d"]))
        names = [n.text.strip() for n in xml.findall("*/Body/Name")]
        if self.body not in names:
            raise RuntimeError(f"rigid body '{self.body}' not in QTM, it has: {names}")
        self.body_index = names.index(self.body)
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)
        await self.connection.stream_frames(frames="allframes", components=["6d"], on_packet=self._on_packet)
        await asyncio.sleep(1.0)
        if self.frames == 0:
            raise RuntimeError("no frames from QTM: is it in preview or capture mode?")
        print(f"QTM streaming ({self.frames} frames in the first second, "
              f"{self.tracked} with '{self.body}' tracked)")

    def _on_packet(self, packet) -> None:
        _, bodies = packet.get_6d()
        position, rotation = bodies[self.body_index]
        values = [position.x, position.y, position.z, *rotation.matrix]
        tracked = not any(math.isnan(v) for v in values)
        self.writer.writerow([packet.timestamp, packet.framenumber, *values, f"{time.time():.6f}"])
        self.frames += 1
        self.tracked += tracked
        self.timestamps.append(packet.timestamp)
        self.is_tracked.append(tracked)

    def markers_on(self) -> None:
        """Tracking before this point is not judged: the markers were not in solid mode yet"""
        self.markers_on_frame = len(self.timestamps)

    async def stop(self) -> dict:
        if self.connection is not None:
            try:
                await self.connection.stream_frames_stop()
            except Exception as e:
                print(f"!! Stopping the QTM stream failed ({type(e).__name__}: {e})")
            self.connection.disconnect()
        if self.file is not None:
            self.file.close()
        return self.stats()

    def stats(self) -> dict:
        """Frame counts and the untracked intervals, where the sync pulses should show up"""
        t = np.array(self.timestamps[self.markers_on_frame:], dtype=np.int64)
        tracked = np.array(self.is_tracked[self.markers_on_frame:], dtype=bool)
        t_first = self.timestamps[0] if self.timestamps else 0
        dropouts = []  # [start, duration] [s], start relative to the first frame of the recording
        i = 0
        while i < len(tracked):
            if not tracked[i]:
                j = i
                while j < len(tracked) and not tracked[j]:
                    j += 1
                end = t[j] if j < len(t) else t[-1]
                dropouts.append([round(float(t[i] - t_first) / 1e6, 3), round(float(end - t[i]) / 1e6, 3)])
                i = j
            else:
                i += 1
        rate = (len(t) - 1) / ((t[-1] - t[0]) / 1e6) if len(t) > 1 and t[-1] > t[0] else 0.0
        return {"file": self.path, "frames": int(self.frames), "tracked": int(self.tracked),
                "rate_hz": round(rate, 1), "markers_on_frame": self.markers_on_frame, "dropouts": dropouts}

    @classmethod
    def from_csv(cls, path: str, markers_on_frame: Optional[int] = None) -> "MocapRecorder":
        """A finished recording read back from its CSV file, for checking it again later"""
        recording = cls("", "", path)
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                recording.timestamps.append(int(row["qtm_timestamp_us"]))
                recording.is_tracked.append(row["x_mm"] != "nan")
        recording.frames = len(recording.timestamps)
        recording.tracked = sum(recording.is_tracked)
        if markers_on_frame is None:
            # Not recorded: assume the markers were solid from the first tracked frame
            markers_on_frame = recording.is_tracked.index(True) if recording.tracked else 0
        recording.markers_on_frame = markers_on_frame
        return recording


def find_sync_groups(dropouts: list[list[float]], pulses: int, pulse_s: float) -> list[float]:
    """Start times of the sync pulse groups among the tracking dropouts.

    A group is `pulses` dropouts that each last about one pulse and start about two pulses apart
    (off, then on). Radio latency makes the off phase a little longer than requested, so both are
    matched with a tolerance. Other dropouts, such as the drone leaving the motion capture volume,
    do not match this pattern.
    """
    candidates = [start for start, duration in dropouts if abs(duration - pulse_s) <= 0.3 * pulse_s]
    groups = []
    i = 0
    while i + pulses <= len(candidates):
        group = candidates[i:i + pulses]
        if all(abs((b - a) - 2 * pulse_s) <= 0.25 * pulse_s * 2 for a, b in zip(group, group[1:])):
            groups.append(group[0])
            i += pulses
        else:
            i += 1
    return groups


def report_mocap(stats: dict, pulses: int, pulse_s: float, expected_groups: int = 2) -> bool:
    print(f"   {stats['frames']} frames at {stats['rate_hz']:.0f} Hz, {stats['tracked']} tracked")
    groups = find_sync_groups(stats["dropouts"], pulses, pulse_s)
    ok = len(groups) == expected_groups
    where = ", ".join(f"{g:.1f} s" for g in groups)
    if ok:
        print(f"OK: {len(groups)} groups of {pulses} sync pulses in the motion capture (at {where})")
    else:
        print(f"!! Found {len(groups)} groups of {pulses} sync pulses in the motion capture, expected "
              f"{expected_groups}" + (f" (at {where})" if groups else ""))
    long_ones = [d for d in stats["dropouts"] if d[1] > 0.1 and abs(d[1] - pulse_s) > 0.3 * pulse_s]
    if long_ones:
        print("   Other tracking dropouts over 0.1 s (start, duration) [s]: "
              + ", ".join(f"({a:.1f}, {b:.2f})" for a, b in long_ones))
    return ok


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


def check_log_file(path: str, expected_events: Optional[int], config: str = DEFAULT_CONFIG,
                   expected_marker_changes: Optional[int] = None) -> list[str]:
    """Check a log file for replay. Returns the problems found; an empty list means the log is usable.

    The SD card can stall while the file is being opened, which drops events in the first second.
    That part is not needed: the replay starts at the estimator reset, after the first sync pulses.
    So gaps in the periodic events only count from the first sync pulse on.
    """
    problems: list[str] = []
    print(f"Checking {path}")

    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 9 or data[0] != 0xBC:
        return ["not a uSD log file (bad magic byte or too short)"]
    if crc32(data[:-4]) != struct.unpack("<I", data[-4:])[0]:
        problems.append("CRC mismatch: the file is truncated or corrupt")

    log = cfusdlog.decode(path)
    if log is None:
        return problems + ["could not decode the file"]

    total = sum(len(event["timestamp"]) for event in log.values())
    if expected_events is not None and total != expected_events:
        problems.append(f"file has {total} events, the drone accepted {expected_events}")

    for name in EXPECTED_PERIODIC_EVENTS.keys() | set(EXPECTED_OTHER_EVENTS):
        if name not in log:
            problems.append(f"no {name} events: check config.txt, the decks and the firmware")

    sync_start = None
    if "activeMarkerModeChanged" in log:
        sync_start = log["activeMarkerModeChanged"]["timestamp"][0]
        n = len(log["activeMarkerModeChanged"]["timestamp"])
        if expected_marker_changes is not None and n != expected_marker_changes:
            problems.append(f"{n} active marker mode changes, expected {expected_marker_changes} (two per sync pulse)")

    if "kalmanReset" in log:
        resets = log["kalmanReset"]["timestamp"]
        if len(resets) != 1:
            problems.append(f"{len(resets)} estimator resets, expected 1 (the estimator reset itself, state out "
                            f"of bounds)")
        elif sync_start is not None and resets[0] < sync_start:
            problems.append("the estimator reset is before the first sync pulse")

    print(f"   {'event':<24} {'count':>8} {'rate [Hz]':>10} {'max gap [ms]':>13} {'before sync':>12}")
    for name, max_gap in EXPECTED_PERIODIC_EVENTS.items():
        if name not in log:
            continue
        t = np.asarray(log[name]["timestamp"])
        if np.any(np.diff(t) < 0):
            problems.append(f"{name}: timestamps go backwards")
        after = t[t >= sync_start] if sync_start is not None else t
        gaps = np.diff(after)
        largest = gaps.max() if len(gaps) else 0.0
        before = np.diff(t[t <= sync_start]) if sync_start is not None else np.array([])
        largest_before = before.max() if len(before) else 0.0
        duration_s = (t[-1] - t[0]) / 1000.0
        rate = (len(t) - 1) / duration_s if duration_s > 0 else 0.0
        flag = "!!" if largest > max_gap else "  "
        print(f"{flag} {name:<24} {len(t):>8} {rate:>10.1f} {largest:>13.1f} {largest_before:>12.1f}")
        if largest > max_gap:
            first = after[1:][gaps > max_gap][0] - t[0]
            problems.append(f"{name}: {int(np.sum(gaps > max_gap))} gaps over {max_gap:.0f} ms after the first "
                            f"sync pulse (first at {first / 1000:.2f} s): events were dropped during the run")

    missing = [f"{event}: {name}" for event, names in read_config_variables(config).items()
               if event in log for name in names if name not in log[event]]
    if missing:
        problems.append(f"variables in {os.path.basename(config)} but not in the log: {', '.join(missing)}")

    for problem in problems:
        print(f"!! {problem}")
    print("USABLE" if not problems else "NOT USABLE")
    return problems


def find_card_dir(card_dir: Optional[str]) -> Optional[str]:
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


def count_events(path: str) -> Optional[int]:
    """Events in a log file, or None if it cannot be decoded (empty or cut off by a failed run)"""
    try:
        log = cfusdlog.decode(path)
    except Exception:
        return None
    return None if log is None else sum(len(event["timestamp"]) for event in log.values())


@dataclass
class Fetch:
    """Copy and check the log files of all runs that do not have theirs yet"""

    out_dir: str = "lh_mocap_runs"
    """Directory with runs.jsonl, and where the log files are copied to"""
    card_dir: Optional[str] = None
    """Mount point of the SD card in the card reader, found under /media/$USER if not given"""
    log_name: str = "lhmocap"
    """Log file name prefix, as on line 3 of config.txt"""
    config: Optional[str] = None
    """config.txt the logs were recorded with, the one on the SD card if not given"""


def fetch_pending(args: Fetch) -> bool:
    """The drone does not report which file it wrote, so files are matched to runs by event count:
    a complete file holds exactly the usd.eventsAccepted recorded for its run."""
    runs_path = os.path.join(args.out_dir, "runs.jsonl")
    with open(runs_path) as f:
        records = [json.loads(line) for line in f if line.strip()]
    pending = [r for r in records if r.get("log_file") is None and r.get("usd")]
    for r in [r for r in pending if r.get("counters_ok") is False]:
        print(f"   {r['run']}: skipped, its uSD counters showed a write error or too few events")
    pending = [r for r in pending if r.get("counters_ok") is not False]
    if not pending:
        print("No runs waiting for their log file")
        return True

    card_dir = find_card_dir(args.card_dir)
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
        r["log_source"] = matches[0]
        r["file_problems"] = check_log_file(target, r["usd"]["eventsAccepted"], config, 4 * r["sync_pulses"])
        r["file_ok"] = not r["file_problems"]
        all_ok &= r["file_ok"]

    for r in pending:
        # Check the motion capture again from its file, so every run is judged by the same rules
        mocap_file = r.get("mocap", {}).get("file")
        if mocap_file and os.path.exists(mocap_file):
            print(f"\n{r['run']}: motion capture")
            stats = MocapRecorder.from_csv(mocap_file, r["mocap"].get("markers_on_frame")).stats()
            r["mocap_ok"] = report_mocap(stats, r["sync_pulses"], r["sync_pulse_s"])

    with open(runs_path, "w") as f:
        f.writelines(json.dumps(r) + "\n" for r in records)

    print("\nSummary:")
    for r in pending:
        if "file_ok" not in r:
            print(f"   {r['run']}: NOT CHECKED (no matching file)")
        elif r["file_ok"] and r.get("mocap_ok", True) and r.get("completed", False):
            print(f"   {r['run']}: USABLE")
        else:
            reasons = list(r.get("file_problems", []))
            if not r.get("mocap_ok", True):
                reasons.append("motion capture sync pulses not found")
            if not r.get("completed", False):
                reasons.append("the run did not complete")
            print(f"   {r['run']}: NOT USABLE ({'; '.join(reasons)})")
    return all_ok


@dataclass
class Check:
    """Check one log file"""

    log_file: tyro.conf.Positional[str]
    """Log file to check"""
    expected_events: Optional[int] = None
    """usd.eventsAccepted read after the flight, to compare with the number of events in the file"""
    config: str = DEFAULT_CONFIG
    """config.txt the log was recorded with, to check that every variable in it was logged"""
    sync_pulses: Optional[int] = None
    """Sync pulses at each end of the run, to check the active marker events"""


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
    """Refuse to start unless the firmware, decks and SD card are set up for this logging"""
    param = cf.param()
    names = set(param.names())
    for deck in ("deck.bcLighthouse4", "deck.bcUSD", "deck.bcActiveMarker"):
        if deck not in names or await param.get(deck) != 1:
            raise RuntimeError(f"{deck} is not attached")
    if "usd.eventsAccepted" not in cf.log().names():
        raise RuntimeError("usd.eventsAccepted not found: flash the firmware of this branch")
    if await param.get("lighthouse.method") != 1:
        raise RuntimeError("lighthouse.method is not 1: the log would have no crossing beams yaw error")
    if not await param.get("usd.canLog"):
        raise RuntimeError("usd.canLog is 0: no SD card, or the config.txt is not valid")


async def read_log_once(cf, names: list[str]) -> dict:
    block = await cf.log().create_block()
    for name in names:
        await block.add_variable(name)
    stream = await block.start(100)
    sample = await asyncio.wait_for(stream.next(), timeout=3.0)
    await stream.stop()
    return sample.data


async def read_usd_stats(cf) -> dict:
    """Read the uSD integrity counters. They are reset when logging starts and hold after it stops."""
    data = await read_log_once(cf, ["usd.eventsRequested", "usd.eventsAccepted", "usd.writeError"])
    return {k.split(".")[1]: v for k, v in data.items()}


def report_usd_stats(stats: dict) -> bool:
    if stats["eventsAccepted"] is None:
        print("!! No uSD counters: the connection was lost before they could be read")
        return False
    ok = True
    requested, accepted = stats["eventsRequested"], stats["eventsAccepted"]
    if requested != accepted:
        # Usually a stall of the SD card while the file is opened; fetch checks where they are
        print(f"   {requested - accepted} of {requested} events dropped. fetch checks that they are all "
              f"before the first sync pulse.")
    else:
        print(f"OK: no dropped events ({accepted} events)")
    minimum = MIN_EVENTS_PER_S * stats.get("logged_s", 0)
    if accepted < minimum:
        print(f"!! TOO FEW EVENTS: {accepted} in {stats['logged_s']:.0f} s of logging, "
              f"expected at least {minimum:.0f}.")
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


async def setup_markers(cf, o: FlyOptions) -> None:
    param = cf.param()
    # Apply mode changes every 10 ms instead of every second, so short sync pulses work
    await param.set("activeMarker.poll", 1)
    for side, value in (("front", o.marker_brightness), ("left", o.marker_brightness),
                        ("right", o.marker_brightness), ("back", 0)):
        await param.set(f"activeMarker.{side}", value)
    await param.set("activeMarker.mode", MARKER_MODE_SOLID)
    # The poll setting itself is only picked up at the deck task's next (1 s) update
    await asyncio.sleep(1.5)


CONTROLLER_LIMITS = {
    "max_speed": ("posCtlPid.xVelMax", "posCtlPid.yVelMax"),
    "max_tilt": ("posCtlPid.rLimit", "posCtlPid.pLimit"),
}


async def raise_controller_limits(cf, o: FlyOptions) -> dict:
    """Set the requested controller limits. Returns the original values, to restore them afterwards."""
    param = cf.param()
    original = {}
    for option, names in CONTROLLER_LIMITS.items():
        value = getattr(o, option)
        if value is None:
            continue
        for name in names:
            original[name] = await param.get(name)
            await param.set(name, value)
        print(f"{', '.join(names)}: {original[names[0]]:g} -> {value:g} for this run")
    return original


async def restore_params(cf, original: dict) -> None:
    for name, value in original.items():
        try:
            await cf.param().set(name, value)
        except Exception as e:
            print(f"!! Could not restore {name} to {value} ({type(e).__name__}: {e}), power-cycle the drone")


async def sync_pulses(cf, o: FlyOptions) -> None:
    param = cf.param()
    print(f"Sync: {o.sync_pulses} marker pulses")
    for _ in range(o.sync_pulses):
        await param.set("activeMarker.mode", MARKER_MODE_OFF)
        await asyncio.sleep(o.sync_pulse_s)
        await param.set("activeMarker.mode", MARKER_MODE_SOLID)
        await asyncio.sleep(o.sync_pulse_s)


async def record_console(cf, path: str, stop: asyncio.Event) -> None:
    # One Console object for the whole run: each new one replays the lines received so far
    console = cf.console()
    with open(path, "a") as f:
        while not stop.is_set():
            for line in await console.get_lines():
                f.write(line + "\n")
            f.flush()
            await asyncio.sleep(0.1)


def save_lh_config(o: FlyOptions) -> Optional[str]:
    """Save the Lighthouse geometry and calibration once per session (output directory)"""
    path = os.path.join(o.out_dir, "lh_config.yaml")
    if os.path.exists(path):
        return path
    print(f"Saving the Lighthouse configuration to {path}")
    result = subprocess.run(["cfcli", "-u", o.uri, "lh", "config", "read", "-o", path])
    if result.returncode != 0 or not os.path.exists(path):
        raise RuntimeError("cfcli lh config read failed; save it by hand to " + path)
    return path


async def fly(p: Pattern, run_name: str) -> dict:
    """Run one pattern. Returns the record for runs.jsonl."""
    record: dict = {"completed": False}
    mocap = MocapRecorder(p.qtm_host, p.qtm_body, os.path.join(p.out_dir, f"{run_name}_mocap.csv"))
    cf = None
    original_params: dict = {}
    console_stop = asyncio.Event()
    console_task = None
    logging_started = False
    logging_stopped = False
    logging_start = 0.0
    logged_s = 0.0
    airborne = False
    try:
        await mocap.start()
        print(f"Connecting to {p.uri}...")
        cf = await Crazyflie.connect_from_uri(LinkContext(), p.uri)
        console_task = asyncio.create_task(
            record_console(cf, os.path.join(p.out_dir, f"{run_name}_console.txt"), console_stop))
        await check_firmware(cf)
        param = cf.param()
        supervisor = cf.supervisor()
        hlc = cf.high_level_commander()

        original_params = await raise_controller_limits(cf, p)
        await setup_markers(cf, p)
        mocap.markers_on()
        record.update(await p.prepare())

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
                               f"writeError {early['writeError']}). Power-cycle the drone and try again.")
        print(f"SD logging OK ({early['eventsAccepted']} events in the first second)")

        await sync_pulses(cf, p)

        print(f"Resetting the estimator, then {p.rest_s:.0f} s at rest")
        await param.set("kalman.resetEstimation", 1)
        await asyncio.sleep(p.rest_s)

        est = await read_log_once(cf, ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z"])
        ground = (est["stateEstimate.x"], est["stateEstimate.y"], est["stateEstimate.z"])
        record["ground_estimate"] = [round(v, 3) for v in ground]
        print(f"Estimated position on the ground: ({ground[0]:+.2f}, {ground[1]:+.2f}, {ground[2]:+.2f})")

        if p.dry_run:
            print(f"Dry run: logging for {p.dry_run_time:.0f} s without arming")
            await asyncio.sleep(p.dry_run_time)
        else:
            if not (p.x_min <= ground[0] <= p.x_max and p.y_min <= ground[1] <= p.y_max):
                raise RuntimeError("the estimated position is outside the flight area, not arming")
            await wait_for(supervisor, lambda s: s.can_be_armed, "can_be_armed")
            print("Arming")
            await supervisor.send_arming_request(True)
            await wait_for(supervisor, lambda s: s.is_armed, "is_armed")
            await wait_for(supervisor, lambda s: s.can_fly, "can_fly")

            height = p.first_height()
            takeoff_s = max(2.0, height / 0.4)
            print(f"Taking off to {height:.2f} m")
            await hlc.take_off(height, None, takeoff_s, None)
            airborne = True
            await asyncio.sleep(takeoff_s + 1.0)

            f = Flight(cf, p, (ground[0], ground[1], height))
            await f.go_to(p.x, p.y, height)
            await f.hold(p.settle_s)
            await p.fly(f)
            # Back to the start point: the middle of the area, where motion capture sees the end pulses
            print(f"To the sync point ({p.x:+.2f}, {p.y:+.2f}, {p.sync_height:.2f})")
            await f.go_to(p.x, p.y, p.sync_height)
            await f.hold(1.0)

        await sync_pulses(cf, p)

        # Stop before landing: SD write errors have been seen during landing and disarm
        print("Stopping SD logging")
        await param.set("usd.logging", 0)
        logged_s = time.monotonic() - logging_start
        logging_stopped = True
        record["completed"] = True

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("Interrupted")
    except Exception as e:
        print(f"!! {type(e).__name__}: {e}")
        record["error"] = f"{type(e).__name__}: {e}"
    finally:
        if cf is not None:
            # Stop logging before landing here too: the touchdown can make the SD card drop events.
            # With a short timeout, so a lost radio link does not delay the landing.
            if logging_started and not logging_stopped:
                print("Stopping SD logging")
                try:
                    await asyncio.wait_for(cf.param().set("usd.logging", 0), timeout=1.0)
                except Exception as e:
                    print(f"!! Could not stop SD logging ({type(e).__name__}: {e})")
                logged_s = time.monotonic() - logging_start
                logging_stopped = True
            if airborne:
                print("Landing")
                try:
                    await cf.high_level_commander().land(0.0, None, 3.0, None)
                    await asyncio.sleep(3.5)
                finally:
                    await cf.high_level_commander().stop(None)
            try:
                await cf.supervisor().send_arming_request(False)
            except Exception:
                pass
            if logging_started:
                # The counters are final once the writer has closed the file
                await asyncio.sleep(1.0)
                try:
                    stats = await read_usd_stats(cf)
                except Exception as e:
                    print(f"!! Could not read the uSD counters ({type(e).__name__}: {e}). Treat this run's log as bad.")
                    stats = {"eventsRequested": None, "eventsAccepted": None, "writeError": None}
                stats["logged_s"] = round(logged_s, 1)
                record["usd"] = stats
            try:
                await cf.param().set("activeMarker.poll", 0)
            except Exception:
                pass
            await restore_params(cf, original_params)
            console_stop.set()
            if console_task is not None:
                await console_task
            try:
                await cf.disconnect()
            except Exception:
                pass
        record["mocap"] = await mocap.stop()
    return record


def run_pattern(p: Pattern) -> None:
    try:
        check_area(p)
    except ValueError as e:
        raise SystemExit(f"!! Not flying: {e}")
    os.makedirs(p.out_dir, exist_ok=True)
    lh_config = save_lh_config(p)
    run_name = f"{time.strftime('%Y%m%d-%H%M%S')}_{p.name}"
    record = asyncio.run(fly(p, run_name))

    counters_ok = None
    if "usd" in record:
        print("\nuSD counters:")
        counters_ok = report_usd_stats(record["usd"])
    print("\nMotion capture:")
    mocap_ok = report_mocap(record["mocap"], p.sync_pulses, p.sync_pulse_s)

    record = {
        "run": run_name,
        "pattern": p.name,
        **record,
        "counters_ok": counters_ok,
        "mocap_ok": mocap_ok,
        "log_file": None,
        "lh_config": lh_config,
        **{k: v for k, v in asdict(p).items() if k != "out_dir"},
    }
    with open(os.path.join(p.out_dir, "runs.jsonl"), "a") as f:
        f.write(json.dumps(record) + "\n")
    print(f"\nRun recorded as {run_name} in {p.out_dir}/runs.jsonl")
    print("After the last run of the session, with the SD card in the reader: uv run fly_lh_mocap_log.py fetch")


def main() -> None:
    subcommands = [Annotated[cls, tyro.conf.subcommand(cls.name, description=cls.__doc__)] for cls in PATTERNS]
    args = tyro.cli(Union[tuple(subcommands + [
        Annotated[Fetch, tyro.conf.subcommand("fetch")],
        Annotated[Check, tyro.conf.subcommand("check")],
    ])])
    if isinstance(args, Fetch):
        raise SystemExit(0 if fetch_pending(args) else 1)
    if isinstance(args, Check):
        changes = 4 * args.sync_pulses if args.sync_pulses is not None else None
        raise SystemExit(1 if check_log_file(args.log_file, args.expected_events, args.config, changes) else 0)
    run_pattern(args)


if __name__ == "__main__":
    main()
