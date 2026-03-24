#!/usr/bin/env python3
"""Fly multiple Crazyflies through local waypoints, then land on their pads."""

import asyncio
from dataclasses import dataclass
from typing import Any

from cflib2 import Crazyflie, LinkContext
from cflib2.toc_cache import FileTocCache


async def drain_log(log_stream, last_log: dict[str, Any]) -> None:
    """Continuously drain a log stream, keeping only the most recent reading."""
    while True:
        data = await log_stream.next()
        last_log["data"] = data.data


# Configuration

URIS = [
    "radio://0/90/2M/ABAD1DEA01",
    # "radio://0/90/2M/ABAD1DEA02",
    # "radio://0/90/2M/ABAD1DEA03",
    # "radio://0/90/2M/ABAD1DEA04",
    # "radio://0/90/2M/ABAD1DEA05",
    # "radio://0/90/2M/ABAD1DEA06",
    # # "radio://0/90/2M/ABAD1DEA07",
    # "radio://0/90/2M/ABAD1DEA08",
    # "radio://0/90/2M/ABAD1DEA09",
]

TAKEOFF_HEIGHT = 1.0  # meters
TAKEOFF_DURATION = 2.0  # seconds
LANDING_DURATION = 6.0  # seconds

LOG_INTERVAL = 100  # ms

WAYPOINT_RADIUS_X = 0.7
WAYPOINT_RADIUS_Y = 0.2
WAYPOINT_RADIUS_Z = 0.5
WAYPOINT_DURATION = 2.0  # seconds per waypoint


@dataclass
class DroneSession:
    cf: Crazyflie
    uri: str
    hlc: Any
    last_log: dict[str, Any]
    log_task: asyncio.Task
    pad_x: float
    pad_y: float
    pad_z: float


async def setup_drone(cf: Crazyflie, uri: str) -> DroneSession:
    hlc = cf.high_level_commander()

    param = cf.param()
    # param.set("landingCrtl.pkp", 5.5)
    # param.set("landingCrtl.pki", 1.9)
    # param.set("landingCrtl.pkd", 1.0)
    param.set("landingCrtl.hOffset", 0.02)
    param.set("landingCrtl.hDuration", 1.0)
    param.set("ctrlMel.ki_z", 1.5)

    # # param.set("landingCrtl.m_pos_kp", 0.4)
    # # param.set("landingCrtl.m_pos_ki", 0.2)
    # # param.set("landingCrtl.m_pos_kd", 0.05)

    # param.set("landingCrtl.m_att_kp", 70000.0)
    # param.set("landingCrtl.m_att_ki", 0.0)
    # param.set("landingCrtl.m_att_kd", 20000.0)

    param.set("landingCrtl.m_pos_kp", 1.0)
    param.set("landingCrtl.m_pos_ki", 0.7)
    param.set("landingCrtl.m_pos_kd", 0.1)

    param.set("landingCrtl.m_att_kp", 100000.0)
    param.set("landingCrtl.m_att_ki", 0.0)
    param.set("landingCrtl.m_att_kd", 10000.0)

    param.set("stabilizer.controller", 2)

    log = cf.log()
    block = await log.create_block()
    await block.add_variable("stateEstimate.x")
    await block.add_variable("stateEstimate.y")
    await block.add_variable("stateEstimate.z")
    await block.add_variable("pm.state")

    log_stream = await block.start(LOG_INTERVAL)
    last_log = {"data": (await log_stream.next()).data}
    log_task = asyncio.create_task(drain_log(log_stream, last_log))

    values = last_log["data"]
    pad_x = values["stateEstimate.x"]
    pad_y = values["stateEstimate.y"]
    pad_z = values["stateEstimate.z"]
    print(f"[{uri}] Pad position: x={pad_x:.3f}  y={pad_y:.3f}  z={pad_z:.3f}")

    return DroneSession(
        cf=cf,
        uri=uri,
        hlc=hlc,
        last_log=last_log,
        log_task=log_task,
        pad_x=pad_x,
        pad_y=pad_y,
        pad_z=pad_z,
    )


async def go_to_waypoint(session: DroneSession, x: float, y: float, z: float, label: str) -> None:
    print(f"[{session.uri}] {label}: x={x:.2f} y={y:.2f} z={z:.2f}")
    await session.hlc.go_to(
        x,
        y,
        z,
        0.0,
        WAYPOINT_DURATION,
        relative=False,
        linear=False,
        group_mask=None,
    )


def print_landing_status(session: DroneSession) -> bool:
    values = session.last_log["data"]
    is_charging = values["pm.state"] == 1
    x = values["stateEstimate.x"]
    y = values["stateEstimate.y"]
    z = values["stateEstimate.z"]
    distance = (
        (x - session.pad_x) ** 2
        + (y - session.pad_y) ** 2
        + (z - session.pad_z) ** 2
    ) ** 0.5
    print(
        f"[{session.uri}] Charging: {is_charging} | "
        f"Distance to pad: {distance:.3f} | "
        f"Position: x={x:.3f} y={y:.3f} z={z:.3f}"
    )
    return is_charging


async def retry_landing(session: DroneSession) -> None:
    await session.cf.platform().send_arming_request(True)
    await asyncio.sleep(1.0)

    await session.hlc.take_off(TAKEOFF_HEIGHT, None, TAKEOFF_DURATION, None)
    await asyncio.sleep(TAKEOFF_DURATION + 1.0)
    
    await session.hlc.go_to(
        session.pad_x,
        session.pad_y,
        session.pad_z + 0.5,
        0.0,
        3.0,
        relative=False,
        linear=False,
        group_mask=None,
    )
    await asyncio.sleep(3.2)
    await session.hlc.land(session.pad_z, None, LANDING_DURATION, None)
    await asyncio.sleep(LANDING_DURATION)

    await session.cf.platform().send_arming_request(False)
    await asyncio.sleep(1.0)


async def main() -> None:
    print(f"\nConnecting to {len(URIS)} drones...")
    ctx = LinkContext()
    cfs = await asyncio.gather(
        *[Crazyflie.connect_from_uri(ctx, uri, FileTocCache('cache')) for uri in URIS]
    )
    print("All connected!")

    sessions: list[DroneSession] = []
    try:
        sessions = await asyncio.gather(
            *[setup_drone(cf, uri) for cf, uri in zip(cfs, URIS)]
        )

        print("\nArming all drones...")
        await asyncio.gather(
            *[session.cf.platform().send_arming_request(True) for session in sessions]
        )
        await asyncio.sleep(1.0)
        print("All armed!")

        print("\nTaking off all drones...")
        await asyncio.gather(
            *[
                session.hlc.take_off(TAKEOFF_HEIGHT, None, TAKEOFF_DURATION, None)
                for session in sessions
            ]
        )
        await asyncio.sleep(TAKEOFF_DURATION + 1.0)

        # Phase 1 – Z axis: back and forth ±0.5 m
        print("\nWaypoints: Z axis...")
        for label, z_off in [("Z+", WAYPOINT_RADIUS_Z), ("Z-", -WAYPOINT_RADIUS_Z)]:
            await asyncio.gather(
                *[
                    go_to_waypoint(
                        s, s.pad_x, s.pad_y,
                        s.pad_z + TAKEOFF_HEIGHT + z_off, label,
                    )
                    for s in sessions
                ]
            )
            await asyncio.sleep(WAYPOINT_DURATION + 0.5)

        # Return to centre
        await asyncio.gather(
            *[
                go_to_waypoint(s, s.pad_x, s.pad_y, s.pad_z + TAKEOFF_HEIGHT, "centre")
                for s in sessions
            ]
        )
        await asyncio.sleep(WAYPOINT_DURATION + 0.5)

        # Phase 2 – X+Y simultaneously: back and forth
        print("\nWaypoints: X+Y axis...")
        for label, x_off, y_off in [
            ("XY+", WAYPOINT_RADIUS_X, WAYPOINT_RADIUS_Y),
            ("XY-", -WAYPOINT_RADIUS_X, -WAYPOINT_RADIUS_Y),
        ]:
            await asyncio.gather(
                *[
                    go_to_waypoint(
                        s, s.pad_x + x_off, s.pad_y + y_off,
                        s.pad_z + TAKEOFF_HEIGHT, label,
                    )
                    for s in sessions
                ]
            )
            await asyncio.sleep(WAYPOINT_DURATION + 0.5)

        # Return to centre
        await asyncio.gather(
            *[
                go_to_waypoint(s, s.pad_x, s.pad_y, s.pad_z + TAKEOFF_HEIGHT, "centre")
                for s in sessions
            ]
        )
        await asyncio.sleep(WAYPOINT_DURATION + 0.5)

        # Phase 3 – Diagonal X+Y+Z simultaneously: back and forth
        print("\nWaypoints: X+Y+Z diagonal...")
        for label, x_off, y_off, z_off in [
            ("diag+", WAYPOINT_RADIUS_X, WAYPOINT_RADIUS_Y, WAYPOINT_RADIUS_Z),
            ("diag-", -WAYPOINT_RADIUS_X, -WAYPOINT_RADIUS_Y, -WAYPOINT_RADIUS_Z),
        ]:
            await asyncio.gather(
                *[
                    go_to_waypoint(
                        s, s.pad_x + x_off, s.pad_y + y_off,
                        s.pad_z + TAKEOFF_HEIGHT + z_off, label,
                    )
                    for s in sessions
                ]
            )
            await asyncio.sleep(WAYPOINT_DURATION + 0.5)

        print("\nFlying to landing start (+1.0m over each pad)...")
        await asyncio.gather(
            *[
                session.hlc.go_to(
                    session.pad_x,
                    session.pad_y,
                    session.pad_z + 1.0,
                    0.0,
                    2.5,
                    relative=False,
                    linear=False,
                    group_mask=None,
                )
                for session in sessions
            ]
        )
        await asyncio.sleep(5.0)

        async def land_with_retry(session: DroneSession) -> None:
            print(f"[{session.uri}] Initial landing attempt.")
            await session.hlc.land(session.pad_z, None, LANDING_DURATION, None)
            await asyncio.sleep(LANDING_DURATION)

            await session.cf.platform().send_arming_request(False)
            await asyncio.sleep(1.0)

            charging = print_landing_status(session)
            attempt = 0
            while not charging:
                attempt += 1
                print(f"[{session.uri}] Retry landing attempt {attempt}...")
                await retry_landing(session)
                charging = print_landing_status(session)

            if attempt == 0:
                print(f"[{session.uri}] Charging confirmed on first attempt.")
            else:
                print(f"[{session.uri}] Charging confirmed after {attempt} retry attempt(s).")

        print("\nLanding all drones simultaneously...")
        await asyncio.gather(*[land_with_retry(session) for session in sessions])

    finally:
        if sessions:
            await asyncio.gather(*[session.hlc.stop(None) for session in sessions])
            print("\nDisarming all drones...")
            await asyncio.gather(
                *[session.cf.platform().send_arming_request(False) for session in sessions]
            )
            for session in sessions:
                session.log_task.cancel()

        print("Disconnecting...")
        await asyncio.gather(*[cf.disconnect() for cf in cfs])
        print("Done!")

if __name__ == "__main__":
    asyncio.run(main())
