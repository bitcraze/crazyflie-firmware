#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "PyYAML~=6.0",
# ]
# ///
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2026 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Simfield: launches one Simmyflie for each local unit of a swarm
described in a yaml file, and stops them all when it is stopped.

Usage:
    uv run tools/sim/simfield.py <swarm.yaml> [--binary=<sim.elf>] [--console]

Swarm file format:

    name: Sim swarm                     # required
    description: Two Simmyflies         # optional
    units:                              # required
    - uri: udp://localhost:50000        # required
      name: sim-01                      # optional
      description: The first one        # optional

Unit settings, all optional, given for the swarm and/or for a unit. A unit
uses its own value when it is given, and otherwise the swarm's value:

    binary: ../build/sim.elf            # relative to the yaml file
    ground_level: 0.0                   # m
    beacon: 239.255.50.0:49999          # <multicast group>:<port>
    start_x: 0.0                        # m
    start_y: 0.0                        # m
    start_yaw: 0                        # degrees

--binary on the command line overrides 'binary' for every unit. Without
either, build/sim.elf of the firmware repo Simfield is in is run.

Unknown fields are ignored. Optional fields that are not given are left to
the Simmyflie's default. Units whose URI is not udp://localhost:<port> or
udp://127.0.0.1:<port> are skipped with a warning.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import urlparse

import yaml

DEFAULT_BINARY = Path(__file__).resolve().parents[2] / "build" / "sim.elf"

LOCAL_HOSTS = ("localhost", "127.0.0.1")
POLL_INTERVAL_S = 0.2
STOP_TIMEOUT_S = 2.0

# Unit settings that are passed on as Simmyflie options. Each can be given
# for the swarm and for a unit; the unit's value wins.
NUMBER_OPTIONS = {
    "ground_level": "--ground-level",
    "start_x": "--start-x",
    "start_y": "--start-y",
    "start_yaw": "--start-yaw",
}
BEACON_OPTION = "--beacon"
TEXT_FIELDS = ("name", "description")


@dataclass
class Unit:
    index: int
    uri: str
    name: str | None
    port: int
    binary: Path
    options: list[str] = field(default_factory=list)

    @property
    def label(self):
        return self.name if self.name is not None else str(self.port)


class SwarmError(Exception):
    def __init__(self, problems):
        super().__init__("\n".join(problems))
        self.problems = problems


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def check_text(container, key, where, problems):
    if key in container and not isinstance(container[key], str):
        problems.append(f"{where}: '{key}' must be a string")


def check_number(container, key, where, problems):
    if key in container and not is_number(container[key]):
        problems.append(f"{where}: '{key}' must be a number")


def check_beacon(container, where, problems):
    if "beacon" not in container:
        return
    beacon = container["beacon"]
    if not isinstance(beacon, str):
        problems.append(f"{where}: 'beacon' must be a string <multicast group>:<port>")
        return
    group, sep, port = beacon.rpartition(":")
    if not sep or not group or not port.isdigit() or not 0 < int(port) < 65536:
        problems.append(f"{where}: 'beacon' must be <multicast group>:<port>, got '{beacon}'")


def check_unit_settings(container, where, problems):
    check_text(container, "binary", where, problems)
    for key in NUMBER_OPTIONS:
        check_number(container, key, where, problems)
    check_beacon(container, where, problems)


def unit_setting(raw, swarm, key):
    """The unit's value of a unit setting, else the swarm's, else None."""
    return raw[key] if key in raw else swarm.get(key)


def parse_local_port(uri):
    """Returns the port of a local UDP URI, or None if the URI is not one.
    Raises ValueError for a local UDP URI with a missing or invalid port."""
    parsed = urlparse(uri)
    if parsed.scheme != "udp" or parsed.hostname not in LOCAL_HOSTS:
        return None
    port = parsed.port  # Raises ValueError when out of range or not a number
    if port is None or port == 0:
        raise ValueError("missing port")
    return port


def unit_where(index, raw):
    if isinstance(raw, dict) and isinstance(raw.get("name"), str):
        return f"unit {index + 1} ({raw['name']})"
    return f"unit {index + 1}"


def load_swarm(path):
    """Reads and validates a swarm file. Returns (name, units, warnings).
    A unit's binary is the one from the file, resolved against the file's
    directory, or DEFAULT_BINARY. Raises SwarmError listing everything that
    is wrong."""
    try:
        with open(path) as f:
            swarm = yaml.safe_load(f)
    except OSError as e:
        raise SwarmError([f"cannot read {path}: {e.strerror}"])
    except yaml.YAMLError as e:
        raise SwarmError([f"{path} is not valid yaml: {e}"])

    if not isinstance(swarm, dict):
        raise SwarmError(["swarm: the file must contain a mapping with 'name' and 'units'"])

    problems = []
    warnings = []

    if "name" not in swarm:
        problems.append("swarm: 'name' is required")
    for key in TEXT_FIELDS:
        check_text(swarm, key, "swarm", problems)
    check_unit_settings(swarm, "swarm", problems)
    base_dir = Path(path).resolve().parent

    raw_units = swarm.get("units")
    if "units" not in swarm:
        problems.append("swarm: 'units' is required")
        raw_units = []
    elif not isinstance(raw_units, list):
        problems.append("swarm: 'units' must be a list")
        raw_units = []

    units = []
    ports = {}  # port -> descriptions of every unit using it, valid or not
    for index, raw in enumerate(raw_units):
        where = unit_where(index, raw)
        if not isinstance(raw, dict):
            problems.append(f"{where}: must be a mapping with at least 'uri'")
            continue

        unit_problems = []
        if "uri" not in raw:
            unit_problems.append(f"{where}: 'uri' is required")
        check_text(raw, "uri", where, unit_problems)
        for key in TEXT_FIELDS:
            check_text(raw, key, where, unit_problems)
        check_unit_settings(raw, where, unit_problems)
        port = None
        if isinstance(raw.get("uri"), str):
            uri = raw["uri"]
            try:
                port = parse_local_port(uri)
            except ValueError:
                unit_problems.append(f"{where}: '{uri}' has no valid port")
            else:
                if port is None:
                    warnings.append(f"{where}: '{uri}' is not a local UDP URI, skipping it")
                    continue
                ports.setdefault(port, []).append(where)
        problems.extend(unit_problems)
        if unit_problems or problems:
            continue

        options = [f"--port={port}"]
        for key, option in NUMBER_OPTIONS.items():
            value = unit_setting(raw, swarm, key)
            if value is not None:
                options.append(f"{option}={value}")
        beacon = unit_setting(raw, swarm, "beacon")
        if beacon is not None:
            options.append(f"{BEACON_OPTION}={beacon}")
        binary = unit_setting(raw, swarm, "binary")
        binary = Path(os.path.normpath(base_dir / binary)) if binary is not None else DEFAULT_BINARY
        units.append(Unit(index, uri, raw.get("name"), port, binary, options))

    for port, clashing in sorted(ports.items()):
        if len(clashing) > 1:
            problems.append(f"port {port} is used by more than one unit: {', '.join(clashing)}")

    if problems:
        raise SwarmError(problems)

    return swarm["name"], units, warnings


def forward_output(stream, prefix):
    for line in stream:
        print(f"{prefix}{line}", end="" if line.endswith("\n") else "\n", flush=True)


def describe_exit(returncode):
    if returncode < 0:
        try:
            return f"was killed by {signal.Signals(-returncode).name}"
        except ValueError:
            return f"was killed by signal {-returncode}"
    return f"exited with code {returncode}"


def missing_binaries(units):
    """Maps each binary that does not exist to the units that use it."""
    missing = {}
    for unit in units:
        if not unit.binary.is_file():
            missing.setdefault(unit.binary, []).append(unit.label)
    return missing


class Simfield:
    def __init__(self, units, console):
        self.units = units
        self.console = console
        self.processes = {}
        self.threads = []
        self.stopping = threading.Event()

    def start(self):
        for unit in self.units:
            process = subprocess.Popen(
                [str(unit.binary)] + unit.options,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE if self.console else subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                errors="replace",
                bufsize=1,
                # Keep Ctrl+C away from the Simmyflies; Simfield stops them.
                start_new_session=True,
            )
            self.processes[unit.port] = (unit, process)
            print(f"Launched {unit.label} on {unit.uri} ({unit.binary})")

            prefix = f"[{unit.label}] "
            streams = [process.stderr] + ([process.stdout] if self.console else [])
            for stream in streams:
                thread = threading.Thread(target=forward_output, args=(stream, prefix), daemon=True)
                thread.start()
                self.threads.append(thread)

    def run(self):
        running = dict(self.processes)
        while not self.stopping.wait(POLL_INTERVAL_S):
            for port, (unit, process) in list(running.items()):
                returncode = process.poll()
                if returncode is not None:
                    print(f"Simmyflie {unit.label} {describe_exit(returncode)}", flush=True)
                    del running[port]

    def stop(self):
        alive = [(unit, p) for unit, p in self.processes.values() if p.poll() is None]
        for _, process in alive:
            process.terminate()
        deadline = time.monotonic() + STOP_TIMEOUT_S
        for _, process in alive:
            try:
                process.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for thread in self.threads:
            thread.join(timeout=1.0)
        if alive:
            print(f"Stopped {len(alive)} Simmyflie(s)")


def main():
    parser = argparse.ArgumentParser(
        description="Launch one Simmyflie for each local unit of a swarm file.")
    parser.add_argument("swarm", type=Path, help="yaml file describing the swarm")
    parser.add_argument("--binary", type=Path,
                        help="Simmyflie binary to run for every unit, overriding 'binary' "
                             f"in the swarm file (default {DEFAULT_BINARY})")
    parser.add_argument("--console", action="store_true",
                        help="print the console output of each Simmyflie")
    args = parser.parse_args()

    try:
        name, units, warnings = load_swarm(args.swarm)
    except SwarmError as e:
        print(f"Error: {args.swarm} is not a valid swarm file, no Simmyflie launched:",
              file=sys.stderr)
        for problem in e.problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    for warning in warnings:
        print(f"Warning: {warning}", file=sys.stderr)

    if args.binary is not None:
        for unit in units:
            unit.binary = args.binary

    missing = missing_binaries(units)
    if missing:
        print("Error: Simmyflie binary not found, no Simmyflie launched:", file=sys.stderr)
        for binary, labels in missing.items():
            print(f"  {binary}: {', '.join(labels)}", file=sys.stderr)
        return 1

    simfield = Simfield(units, args.console)

    def request_stop(signum, frame):
        simfield.stopping.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, request_stop)

    print(f"Swarm '{name}': launching {len(units)} Simmyflie(s), Ctrl+C to stop")
    try:
        simfield.start()
        simfield.run()
    finally:
        simfield.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
