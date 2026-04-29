#!/usr/bin/env python3
""" """

import asyncio
import os
import random
import time
from typing import Any

import yaml
from cflib2 import Crazyflie, LinkContext


async def drain_log(log_stream, last_log: dict[str, Any]) -> None:
    """Continuously drain a log stream, keeping only the most recent reading."""
    while True:
        data = await log_stream.next()
        last_log["data"] = data.data


# Configuration

URI = "radio://0/88/2M/ABAD1DEA01"   # D91F700100  84

TAKEOFF_HEIGHT = 1.0  # meters
TAKEOFF_DURATION = 2.0  # seconds
LANDING_HEIGHT = 1.0  # meters
LANDING_DURATION = 4.0 # 0.1  # seconds

FLY = False

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
    cf = await Crazyflie.connect_from_uri(ctx, URI)
    print("Connected!")

    param = cf.param()
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

    hlc = cf.high_level_commander()
    log_task = None
    try:
        # Set up logging (position + power management)
        log = cf.log()
        block = await log.create_block()
        await block.add_variable("stateEstimate.x")
        await block.add_variable("stateEstimate.y")
        await block.add_variable("stateEstimate.z")
        await block.add_variable("pm.state")
        log_stream = await block.start(100)

        # Start background task to drain log, always keeping the latest reading
        last_log = {"data": (await log_stream.next()).data}
        log_task = asyncio.create_task(drain_log(log_stream, last_log))

        # Give it a moment to catch up past buffered readings
        await asyncio.sleep(0.2)

        # Save pad position from latest reading
        v = last_log["data"]
        pad_x = v["stateEstimate.x"]
        pad_y = v["stateEstimate.y"]
        pad_z = v["stateEstimate.z"] + 0.0
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
        trials = 3
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
            await asyncio.sleep(3.0)

            # # Apply landing PID overrides
            # for name, value in landing_pid.items():
            #     if value is not None:
            #         await param.set(name, value)
            # print("Landing PID applied")

            # # Go to landing start
            # await hlc.go_to(
            #     pad_x,
            #     pad_y,
            #     pad_z + 0.5,
            #     0.0,
            #     2.5,
            #     relative=False,
            #     linear=False,
            #     group_mask=None,
            # )
            # await asyncio.sleep(3.0)

            # # Go to landing start
            # await hlc.go_to(
            #     pad_x,
            #     pad_y,
            #     pad_z + landing_offset + 0.0,
            #     0.0,
            #     3.0,
            #     relative=False,
            #     linear=False,
            #     group_mask=None,
            # )
            # await asyncio.sleep(5.0)

            # # Wait until close enough to pad
            # for landing_wait_trials in range(50):
            #     v = last_log["data"]
            #     x = v["stateEstimate.x"]
            #     y = v["stateEstimate.y"]
            #     z = v["stateEstimate.z"]
            #     distance = (x - pad_x)**2 + (y - pad_y)**2
            #     if distance < 0.02:
            #         break
            #     await asyncio.sleep(0.1)

            # Land
            await hlc.land(pad_z, None, LANDING_DURATION, None)
            await asyncio.sleep(LANDING_DURATION)

            # # Restore default PID
            # for name, value in default_pid.items():
            #     await param.set(name, value)
            # print("Default PID restored")

            # Check if charging after 1 second
            await asyncio.sleep(1.0)
            is_charging = last_log["data"]["pm.state"] == 1
            print(f"Charging: {is_charging}")

            v = last_log["data"]
            x = v["stateEstimate.x"]
            y = v["stateEstimate.y"]
            z = v["stateEstimate.z"]
            distance = ((x - pad_x)**2 + (y - pad_y)**2 + (z - pad_z)**2)**0.5
            print(f"Distance to pad: {distance:.3f}")
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
