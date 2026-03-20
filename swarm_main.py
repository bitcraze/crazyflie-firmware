#!/usr/bin/env python3
"""Fly multiple Crazyflies through local waypoints, then land on their pads."""

import asyncio
from dataclasses import dataclass
from typing import Any

from cflib2 import Crazyflie, LinkContext


async def drain_log(log_stream, last_log: dict[str, Any]) -> None:
    """Continuously drain a log stream, keeping only the most recent reading."""
    while True:
        data = await log_stream.next()
        last_log["data"] = data.data


# Configuration

URIS = [
    "radio://0/88/2M/D91F700101",
    "radio://0/88/2M/D91F700102",
    # "radio://0/84/2M/D91F700103",
    # "radio://0/84/2M/D91F700104",
    # "radio://0/84/2M/D91F700105",
    # "radio://0/84/2M/D91F700106",
]

TAKEOFF_HEIGHT = 1.0  # meters
TAKEOFF_DURATION = 2.0  # seconds
LANDING_DURATION = 6.0  # seconds

LOG_INTERVAL = 100  # ms

WAYPOINT_COUNT = 3
WAYPOINT_X_RADIUS = 0.5


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


async def send_waypoint(session: DroneSession, index: int) -> None:
    x_offset = WAYPOINT_X_RADIUS if index % 2 == 1 else -WAYPOINT_X_RADIUS
    x = session.pad_x + x_offset
    y = session.pad_y
    z = session.pad_z + TAKEOFF_HEIGHT
    yaw = 0.0
    print(
        f"[{session.uri}] Waypoint {index}/{WAYPOINT_COUNT}: "
        f"x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.0f}"
    )
    await session.hlc.go_to(
        x,
        y,
        z,
        yaw,
        2.0,
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
        *[Crazyflie.connect_from_uri(ctx, uri) for uri in URIS]
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

        for waypoint_idx in range(1, WAYPOINT_COUNT + 1):
            await asyncio.gather(
                *[
                    send_waypoint(session, waypoint_idx)
                    for session in sessions
                ]
            )
            await asyncio.sleep(2.5)

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

        print("\nLanding drones sequentially (initial attempt)...")
        charging_by_uri: dict[str, bool] = {}
        for index, session in enumerate(sessions, start=1):
            print(f"\nDrone {index}/{len(sessions)} [{session.uri}] initial landing attempt.")
            await session.hlc.land(session.pad_z, None, LANDING_DURATION, None)
            await asyncio.sleep(LANDING_DURATION)

            await session.cf.platform().send_arming_request(False)
            await asyncio.sleep(1.0)

            charging_by_uri[session.uri] = print_landing_status(session)

        print("\nRetrying non-charging drones sequentially...")
        for index, session in enumerate(sessions, start=1):
            if charging_by_uri[session.uri]:
                print(f"\nDrone {index}/{len(sessions)} [{session.uri}] already charging.")
                continue

            print(f"\nDrone {index}/{len(sessions)} [{session.uri}] requires retry.")
            attempt = 0
            while not charging_by_uri[session.uri]:
                attempt += 1
                print(f"[{session.uri}] Retry landing attempt {attempt}...")
                await retry_landing(session)
                charging_by_uri[session.uri] = print_landing_status(session)

            print(f"[{session.uri}] Charging confirmed after {attempt} retry attempt(s).")

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
