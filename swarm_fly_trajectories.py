#!/usr/bin/env python3
"""Fly multiple Crazyflies through a CSV trajectory using cubic spline interpolation.

CSV format
----------
Columns: x0, y0, z0, x1, y1, z1, ..., xN, yN, zN
Each row is a waypoint; column suffix n identifies the drone (0-based).

The script will auto-generate an example CSV if the configured file is missing.
"""

import asyncio
from dataclasses import dataclass
from typing import Any

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline

from cflib2 import Crazyflie, LinkContext
from cflib2.memory import Poly, Poly4D
from cflib2.toc_cache import FileTocCache

starting_positions = [[1.0,1.0,1.0], [1.0, 0.0, 1.0], [1.0, -1.0, 1.0],
                      [0.0, 1.0, 1.0], [0.0, 0.0, 1.0], [0.0, -1.0, 1.0],
                      [-1.0, 1.0, 1.0], [-1.0, 0.0, 1.0], [-1.0, -1.0, 1.0]]

# ---------------------------------------------------------------------------
# Example trajectory generator
# ---------------------------------------------------------------------------

def generate_example_csv(path: str, n_drones: int = 1, n_waypoints: int = 21) -> None:
    """Generate an example CSV with columns x0,y0,z0, x1,y1,z1, ...

    The trajectory is a relative circle in XY with a gentle Z oscillation.
    It starts and ends at (0, 0, 0) so it works cleanly with relative=True.
    """
    t = np.linspace(0, 4 * np.pi, n_waypoints, endpoint=True)
    data: dict[str, np.ndarray] = {}
    for n in range(n_drones):
        # Full circle: sin(0)=0, cos(0)-1=0  →  returns to origin
        data[f"x{n}"] = 0.2 * np.sin(t) + starting_positions[n][0]
        data[f"y{n}"] = 0.2 * np.cos(t) + starting_positions[n][1]
        data[f"z{n}"] = 0.0 + starting_positions[n][2] 
    pd.DataFrame(data).to_csv(path, index=False)
    print(f"Example trajectory saved to {path}")


# ---------------------------------------------------------------------------
# Spline → Poly4D conversion
# ---------------------------------------------------------------------------

SEGMENT_DURATION = 1.0  # seconds per waypoint interval


def waypoints_to_poly4d(
    waypoints_x: np.ndarray,
    waypoints_y: np.ndarray,
    waypoints_z: np.ndarray,
) -> list[Poly4D]:
    """Fit cubic splines through the waypoints and return Poly4D segments.

    SciPy CubicSpline stores segment i coefficients as::

        spline.c[:, i] = [c3, c2, c1, c0]  (descending powers)

    The Crazyflie Poly expects ascending powers::

        [c0, c1, c2, c3, c4, c5, c6, c7]

    So we reverse the four cubic coefficients and pad with zeros to reach the
    required 7th-order polynomial length of 8 values.
    """
    n = len(waypoints_x)
    t = np.arange(n, dtype=float)  # unit-spaced knots → each segment lasts 1 s

    spline_x = CubicSpline(t, waypoints_x)
    spline_y = CubicSpline(t, waypoints_y)
    spline_z = CubicSpline(t, waypoints_z)

    segments: list[Poly4D] = []
    for i in range(n - 1):
        # Reverse descending → ascending, then zero-pad to 8 coefficients
        cx = list(spline_x.c[::-1, i]) + [0.0] * 4
        cy = list(spline_y.c[::-1, i]) + [0.0] * 4
        cz = list(spline_z.c[::-1, i]) + [0.0] * 4
        cyaw = [0.0] * 8
        segments.append(
            Poly4D(SEGMENT_DURATION, Poly(cx), Poly(cy), Poly(cz), Poly(cyaw))
        )

    return segments


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

TRAJECTORY_CSV = "swarm_trajectory.csv"

URIS = [
    "radio://0/90/2M/ABAD1DEA01",
    "radio://0/90/2M/ABAD1DEA02",
    # "radio://0/90/2M/ABAD1DEA03",
    "radio://0/90/2M/ABAD1DEA04",
    # "radio://0/90/2M/ABAD1DEA05",
    "radio://0/90/2M/ABAD1DEA06",
    "radio://0/90/2M/ABAD1DEA07",
    "radio://0/90/2M/ABAD1DEA08",
    "radio://0/90/2M/ABAD1DEA09",
]

TAKEOFF_HEIGHT = 1.0    # metres
TAKEOFF_DURATION = 2.0  # seconds
LANDING_DURATION = 5.0  # seconds
TRAJECTORY_ID = 1


# ---------------------------------------------------------------------------
# Per-drone session
# ---------------------------------------------------------------------------

LOG_INTERVAL = 100  # ms


@dataclass
class DroneSession:
    cf: Crazyflie
    uri: str
    hlc: Any
    total_duration: float
    pad_x: float
    pad_y: float
    pad_z: float


async def setup_drone(
    cf: Crazyflie,
    uri: str,
    trajectory: list[Poly4D],
    total_duration: float,
) -> DroneSession:
    hlc = cf.high_level_commander()

    # Read the pad position before doing anything else
    log = cf.log()
    block = await log.create_block()
    await block.add_variable("stateEstimate.x")
    await block.add_variable("stateEstimate.y")
    await block.add_variable("stateEstimate.z")
    log_stream = await block.start(LOG_INTERVAL)
    values = (await log_stream.next()).data
    pad_x = float(values["stateEstimate.x"])
    pad_y = float(values["stateEstimate.y"])
    pad_z = float(values["stateEstimate.z"])
    print(f"[{uri}] Pad position: x={pad_x:.3f}  y={pad_y:.3f}  z={pad_z:.3f}")

    print(f"[{uri}] Uploading trajectory ({len(trajectory)} segments)...")
    bytes_written = await cf.memory().write_trajectory(trajectory)
    print(f"[{uri}] Uploaded {bytes_written} bytes")

    await hlc.define_trajectory(TRAJECTORY_ID, 0, len(trajectory), 0)
    print(f"[{uri}] Trajectory {TRAJECTORY_ID} defined")

    return DroneSession(cf=cf, uri=uri, hlc=hlc, total_duration=total_duration,
                        pad_x=pad_x, pad_y=pad_y, pad_z=pad_z)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def main() -> None:
    # Load or auto-generate trajectory CSV
    # try:
    #     df = pd.read_csv(TRAJECTORY_CSV)
    #     print(f"Loaded trajectory from {TRAJECTORY_CSV}")
    # except FileNotFoundError:
    #     print(f"{TRAJECTORY_CSV} not found – generating example...")
    generate_example_csv(TRAJECTORY_CSV, n_drones=len(URIS))
    df = pd.read_csv(TRAJECTORY_CSV)

    n_drones = len(URIS)
    total_duration = (len(df) - 1) * SEGMENT_DURATION
    print(f"Trajectory: {len(df) - 1} segments, {total_duration:.1f} s total")

    # Build per-drone Poly4D lists and record each drone's first waypoint
    drone_trajectories: list[list[Poly4D]] = []
    start_positions: list[tuple[float, float, float]] = []
    for n in range(n_drones):
        xs = df[f"x{n}"].values
        ys = df[f"y{n}"].values
        zs = df[f"z{n}"].values
        segs = waypoints_to_poly4d(xs, ys, zs)
        drone_trajectories.append(segs)
        start_positions.append((float(xs[0]), float(ys[0]), float(zs[0])))

    # Connect
    print(f"\nConnecting to {n_drones} drone(s)...")
    ctx = LinkContext()
    cfs: list[Crazyflie] = list(
        await asyncio.gather(
            *[Crazyflie.connect_from_uri(ctx, uri, FileTocCache("cache")) for uri in URIS]
        )
    )
    print("All connected!")

    for cf in cfs:
        param = cf.param()
        # param.set("landingCrtl.pkp", 5.5)
        # param.set("landingCrtl.pki", 1.9)
        # param.set("landingCrtl.pkd", 1.0)
        param.set("landingCrtl.hOffset", 0.02)
        param.set("landingCrtl.hDuration", 1.0)
        param.set("ctrlMel.ki_z", 1.5)
        param.set("stabilizer.controller", 2)

        # param.set("landingCrtl.m_pos_kp", 1.0)
        # param.set("landingCrtl.m_pos_ki", 0.7)
        # param.set("landingCrtl.m_pos_kd", 0.1)

        # param.set("landingCrtl.m_att_kp", 100000.0)
        # param.set("landingCrtl.m_att_ki", 0.0)
        # param.set("landingCrtl.m_att_kd", 10000.0)

    sessions: list[DroneSession] = []
    try:
        sessions = list(
            await asyncio.gather(
                *[
                    setup_drone(cf, uri, traj, total_duration)
                    for cf, uri, traj in zip(cfs, URIS, drone_trajectories)
                ]
            )
        )

        # Arm
        print("\nArming all drones...")
        await asyncio.gather(
            *[s.cf.platform().send_arming_request(True) for s in sessions]
        )
        await asyncio.sleep(1.0)

        # Take off
        print("Taking off...")
        await asyncio.gather(
            *[s.hlc.take_off(TAKEOFF_HEIGHT, None, TAKEOFF_DURATION, None) for s in sessions]
        )
        await asyncio.sleep(TAKEOFF_DURATION + 1.0)

        # Move to the starting position of the trajectory (relative to hover)
        print("Moving to trajectory start position...")
        GO_TO_START_DURATION = 2.0
        await asyncio.gather(
            *[
                s.hlc.go_to(
                    start_positions[i][0],
                    start_positions[i][1],
                    start_positions[i][2],
                    0.0,
                    GO_TO_START_DURATION,
                    relative=False,
                    linear=False,
                    group_mask=None,
                )
                for i, s in enumerate(sessions)
            ]
        )
        await asyncio.sleep(GO_TO_START_DURATION + 0.5)

        # Start trajectory (relative=True so each drone flies from its own position)
        print("Starting trajectory...")
        await asyncio.gather(
            *[
                s.hlc.start_trajectory(TRAJECTORY_ID, 1.0, False, False, False, None)
                for s in sessions
            ]
        )
        await asyncio.sleep(total_duration + 0.5)

        # Return above each pad before landing
        print("Returning to pad positions...")
        GO_TO_PAD_DURATION = 3.0
        await asyncio.gather(
            *[
                s.hlc.go_to(
                    s.pad_x,
                    s.pad_y,
                    s.pad_z + TAKEOFF_HEIGHT,
                    0.0,
                    GO_TO_PAD_DURATION,
                    relative=False,
                    linear=False,
                    group_mask=None,
                )
                for s in sessions
            ]
        )
        await asyncio.sleep(GO_TO_PAD_DURATION + 0.5)

        # Land
        print("Landing...")
        await asyncio.gather(
            *[s.hlc.land(s.pad_z, None, LANDING_DURATION, None) for s in sessions]
        )
        await asyncio.sleep(LANDING_DURATION + 0.5)

        # Stop high-level commander
        await asyncio.gather(*[s.hlc.stop(None) for s in sessions])

    finally:
        if sessions:
            print("\nDisarming all drones...")
            await asyncio.gather(
                *[s.cf.platform().send_arming_request(False) for s in sessions]
            )

        print("Disconnecting...")
        await asyncio.gather(*[cf.disconnect() for cf in cfs])
        print("Done!")


if __name__ == "__main__":
    asyncio.run(main())
