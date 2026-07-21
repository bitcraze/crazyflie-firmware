# Self-Healing Drone Protocol App for Crazyflie 2.x

This folder contains an out-of-tree application layer for the Crazyflie, implementing a distributed self-healing network protocol. The algorithm is based on a virtual spring-damper model designed to autonomously restore connectivity among a swarm of drones.

## Project Structure

| File | Description |
| :--- | :--- |
| `self_healing.c` | Main control loop, P2P message handling, and flight controller integration. |
| `self_healing_math.c/h` | Computation of attractive/repulsive force vectors and conversion to velocity setpoints. |
| `self_healing_state.c/h` | Neighbor Table management and dynamic state tracking. |

## Core Mechanics

Each drone executes a periodic control loop (250ms) where it performs the following:

1. **Periodic Broadcast**: Transmits its current state (`POSITION_UPDATE` or `HELP_PROXY`), ID, and calculated hop count via P2P. Neighbors update their local Neighbor Table upon reception.

2. **Hop Discovery**: Each drone dynamically estimates its distance from the base by identifying the minimum hop count among its neighbors and adding 1, bypassing the need for explicit network flooding.

3. **Failure Detection & Failsafe**: If no ACK is received from the Base Station (ID 0) for more than 2.0 seconds, the drone enters `mission_mode` and broadcasts a `HELP_PROXY` flag.

4. **Force Calculation**: In `mission_mode`, the drone calculates attractive forces towards adjacent hops (h-1 and h+1) and repulsive forces against neighbors violating the safe distance, converting the net force into 2D velocity setpoints.

## Hardware Configuration

- Three Crazyflie 2.x drones configured with unique radio addresses: `E7E7E7E7E0` (Base/ID 0), `E7E7E7E7E1` (ID 1), and `E7E7E7E7E2` (ID 2).
- Altitude sensors (Flow Deck) mounted on the relay drones to maintain a stable Z-axis during the flight test.
- Physical indoor setup: Drones placed in a straight-line topology, maintaining a ~1m separation, all facing the same positive X direction. Newspaper pages were intentionally placed on the floor beneath the flight path to provide high-contrast visual patterns for the optical flow sensors. This ensures positional stability and prevents drift caused by reflective or featureless indoor floor surfaces.

## Visual Feedback (LED Status)

| Status | LED Color |
| :--- | :--- |
| Normal Flight, Hop 0 (Base) | Blue |
| Normal Flight, Hop 1 | Green |
| Normal Flight, Hop 2 | Red |
| Uninitialized / Hop missing | Off |
| Mission Mode (Emergency) | Solid Red (All nodes) |

## Test Execution

1. Power on Drone ID 0 (Base Station, remains on the ground).
2. Power on and take-off Drone ID 1 and ID 2.
3. Connect `cfclient` to Drone ID 1 and verify the `PROVA ->` console logs to ensure the Neighbor Table is correctly populated.
4. Simulate the network failure by either physically powering off Drone ID 0 or setting the `sig_loss = 1` parameter via `cfclient`.
5. Observe the nodes executing the physical repositioning maneuver, driven by the calculated virtual forces to autonomously restore the network topology.

## Known Limitations & Testing Adaptations

* **RSSI Distance Estimation**: Indoor multipath fading causes significant RSSI fluctuations (±10 dBm), making real-time distance estimation unreliable for fine motor control. To demonstrate the physical actuation of the vectors safely, the `rssi_to_distance` function currently utilizes a hardcoded scalar distance of 1.2m. The original log-distance propagation formula is preserved in the codebase.

* **Deterministic Angle Assignment**: Due to the lack of UWB/AoA hardware, the exact heading towards neighbors cannot be dynamically resolved. A deterministic angle mapping based on Drone IDs is implemented specifically for the linear topology tested.
 
* **Telemetry Interferences**: Connecting the `cfclient` via Crazyradio PA for real-time telemetry can cause packet drops in the P2P network.
