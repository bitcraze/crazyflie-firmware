#!/usr/bin/env python3
""" """

import asyncio
import os
import random
import time
from typing import Any

import yaml
from cflib2 import Crazyflie, FileTocCache, LinkContext


async def drain_log(log_stream, last_log: dict[str, Any]) -> None:
    """Continuously drain a log stream, keeping only the most recent reading."""
    while True:
        data = await log_stream.next()
        last_log["data"] = data.data


# Configuration

URI = "radio://0/84/2M/D91F700101"  

TAKEOFF_HEIGHT = 1.0  # meters
TAKEOFF_DURATION = 2.0  # seconds
LANDING_HEIGHT = 1.0  # meters
LANDING_DURATION = 6.0 # 0.1  # seconds

FLY = False

LOG_INTERVAL = 100  # ms

PARAMS_PATH = "params.yaml"

# Landing PID overrides (None = keep default)
LANDING_POS_PID = {
    "posCtlPid.xKp": 2.1,
    "posCtlPid.xKi": 0.0,
    "posCtlPid.xKd": None,
    "posCtlPid.yKp": 2.1,
    "posCtlPid.yKi": 0.0,
    "posCtlPid.yKd": None,
    "posCtlPid.zKp": 1.9,
    "posCtlPid.zKi": 0.1,
    "posCtlPid.zKd": None,
}


async def main() -> None:
    # Connect to drone
    print(f"\nConnecting to drone at {URI}...")
    ctx = LinkContext()
    cache = FileTocCache("cache")
    cf = await Crazyflie.connect_from_uri(ctx, URI, cache)
    print("Connected!")
    hlc = cf.high_level_commander()
    log_task = None
    param = cf.param()
    param.set("landingCrtl.kp", 6.0)
    param.set("landingCrtl.ki", 2.0)
    param.set("landingCrtl.kd", 1.0)
    param.set("landingCrtl.hOffset", 0.08)
    param.set("landingCrtl.hDuration", 2.0)

    try:
        # Set up logging (position + power management)
        log = cf.log()
        block = await log.create_block()
        await block.add_variable("stateEstimate.x")
        await block.add_variable("stateEstimate.y")
        await block.add_variable("stateEstimate.z")
        await block.add_variable("pm.state")

        log_stream = await block.start(LOG_INTERVAL)
        last_log = {"data": (await log_stream.next()).data}
        log_task = asyncio.create_task(drain_log(log_stream, last_log))

        # data = await log_stream.next()
        # timestamp = data.timestamp
        values = last_log["data"]

        pad_x = values["stateEstimate.x"]
        pad_y = values["stateEstimate.y"]
        pad_z = values["stateEstimate.z"]
        print(f"Pad position: x={pad_x:.3f}  y={pad_y:.3f}  z={pad_z:.3f}")

        # Arm drone
        print("\nArming...")
        await cf.platform().send_arming_request(True)
        await asyncio.sleep(1.0)
        print("Armed!")

        rng = random.Random(42)
        distances = []
        num_waypoints = 5
        success_count = 0
        trials = 4
        for trial in range(1, trials + 1):
            # Take off
            print(f"\n{trial}/{trials} Taking off...")
            await hlc.take_off(TAKEOFF_HEIGHT, None, TAKEOFF_DURATION, None)
            await asyncio.sleep(TAKEOFF_DURATION + 1.0)

            # Fly to pseudorandom positions (fixed seed for repeatability)
            for i in range(num_waypoints):
                rx = rng.uniform(-1.3, 1.3)
                ry = rng.uniform(-1.3, 1.3)
                rz = rng.uniform(0.5, 2.0)
                yaw = rng.uniform(0, 360)
                print(
                    f"Waypoint {i+1}/{num_waypoints}: x={rx:.2f} y={ry:.2f} z={rz:.2f} yaw={yaw:.0f}"
                )
                await hlc.go_to(
                    rx,
                    ry,
                    rz,
                    yaw,
                    2.0,
                    relative=False,
                    linear=False,
                    group_mask=None,
                )
                await asyncio.sleep(2.5)

            # Go to landing start
            await hlc.go_to(
                pad_x,
                pad_y,
                pad_z + 1.0,
                0.0,
                2.5,
                relative=False,
                linear=False,
                group_mask=None,
            )
            await asyncio.sleep(5.0)

            # Land
            await hlc.land(pad_z, None, LANDING_DURATION, None)
            await asyncio.sleep(LANDING_DURATION)

            
            # Check if charging after 1 second
            await asyncio.sleep(1.0)
            values = last_log["data"]
            is_charging = values["pm.state"] == 1
            print(f"Charging: {is_charging}")

            
            x = values["stateEstimate.x"]
            y = values["stateEstimate.y"]
            z = values["stateEstimate.z"]
            distance = ((x - pad_x)**2 + (y - pad_y)**2 + (z - pad_z)**2)**0.5
            print(f"Distance to pad: {distance:.3f}")
            print(f"Position: x={x:.3f}  y={y:.3f}  z={z:.3f}")
            print(f"Pad position: x={pad_x:.3f}  y={pad_y:.3f}  z={pad_z:.3f}")
            distances.append(distance)


            if is_charging:
                success_count += 1
            print(f"Current success rate: {success_count}/{trial} = {success_count/trial:.2%}")

    finally:
        if log_task:
            log_task.cancel()
        # Stop high level commander
        await hlc.stop(None)
        # Disarm
        print("\nDisarming...")
        await cf.platform().send_arming_request(False)
        # Disconnect
        print("Disconnecting...")
        await cf.disconnect()
        print("Done!")

if __name__ == "__main__":
    asyncio.run(main())
