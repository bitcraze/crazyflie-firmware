# Self-Healing Drone Protocol App for Crazyflie 2.x

This folder contains an out-of-tree application layer for the Crazyflie, implementing a distributed self-healing network protocol. The algorithm is based on a virtual spring-damper model designed to autonomously restore  connectivity among a swarm of drones.

## Overview

Drones equipped with this firmware act as relay nodes. They continuously exchange their status, `hop_count`, and `id` via P2P broadcast messages. 
The system is purely reactive:
1. *Trigger Phase*: If a drone loses connection to the Base Station (ID 0) for more than 1.5 seconds, it enters `mission_mode` and broadcasts a `HELP_PROXY` packet.
2. *Propagation*: Any neighbor receiving a `HELP_PROXY` packet instantly enters `mission_mode` as well.
3. *Actuation:* Once in mission mode, drones calculate virtual attractive and repulsive forces based on the RSSI of their neighbors. These forces are converted into 2D velocity setpoints (`modeVelocity`), causing the swarm to autonomously expand and physically reposition itself to restore the radio link.

## Setup & Testing

Two or more Crazyflies need to be flashed with this program. 
- Ensure all drones are on the exact same radio channel.
- Each drone must have a unique radio address/ID configured.
- To simulate the Base Station, one node (or a PC via radio) should broadcast packets with ID 0. If this is absent, the drones will timeout after 1.5s and trigger the swarm behavior.

You can read debug messages, including calculated distances and active hop counts, in the console tab of the [cfclient](https://github.com/bitcraze/crazyflie-clients-python).

## Limitations

1. **Radio Interferences:** Since P2P communication happens asynchronously on the radio, connecting a PC to the Crazyflies via the Crazyradio PA for telemetry can cause heavy packet drops. It is highly recommended to connect the Crazyflies using the USB port for debugging.
2. **Omnidirectional Antennas (AoA constraint):** Due to the lack of UWB (Ultra-Wideband) hardware for Time-of-Flight and Angle of Arrival (AoA) measurements, this specific test implementation extracts the scalar distance from the Radio RSSI. To allow 2D vector decomposition for the motors, a deterministic "fake angle" approach based on the drone ID is temporarily utilized for force projection.