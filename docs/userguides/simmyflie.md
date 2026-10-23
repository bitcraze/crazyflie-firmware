---
title: Simmyflie
page_id: simmyflie
---

The Simmyflie is a simulated Crazyflie. It is the normal Crazyflie firmware,
compiled to run as a program on your computer instead of on the drone. The
hardware (sensors, motors, radio) is replaced by a physics engine, and the
Simmyflie talks CRTP over UDP. To cflib and cfclient it looks like a real
Crazyflie: you can connect to it, read logs, set parameters and fly it with
the same scripts you use for a real one.

Several Simmyflies can run at the same time, each one as its own program on
its own UDP port.

## Concepts

* **Instance**: a running Simmyflie. Each instance is identified by the UDP
  port it listens on, by default 50000. You connect to it with the URI
  `udp://localhost:<port>`.
* **World**: the single frame shared by all Simmyflies on the computer. Two
  Simmyflies with the same start pose stand in the same spot. Simmyflies do
  not interact: they fly through each other.
* **Start pose**: the x, y and yaw where a Simmyflie stands on the ground
  before it flies.
* **Ground level**: the height below which a Simmyflie cannot go. It is 0 m
  by default. A Simmyflie always starts at ground level.
* **Status**: The Simmyflie status represent the physical status. It is separate from any other firmware state, and unknown to the normal firmware modules. A Simmyflie is always in one of three states.
  * *At rest*: on the ground, not moving and level.
  * *Flying*: anything else that is not crashed, including a hover.
  * *Crashed*: it hit the ground too fast (default 4 m/s) or too tilted
    (default 45° in pitch or roll). A crashed Simmyflie freezes where it hit
    the ground until it is moved to a start pose.
* **True state**: what the Simmyflie is actually doing, as opposed to what its
  firmware estimates.
* **Beacon**: a small message each Simmyflie sends periodically to anyone on
  the computer who listens, with its pose and status. It does not use a CRTP
  connection. Viewers such as the simviewer use beacons.
* **Positioning**: The Simmyflie has perfect positioning, in the sense that 
  the Kalman filter is fed perfect position and pose measurements, as well as
  perfect IMU data.
* **Battery**: The Simmyflie has infinite battery, and will report a constant
  4.2 V.

## Building

Configure the firmware just as for a real Crazyflie, then build the Simmyflie
from that configuration with `make sim`:

```sh
make cf21bl_defconfig  # or cf2_defconfig, bolt_defconfig, ...
make menuconfig        # optional
make sim
```

The result is `build/sim.elf`. `make sim` uses your configuration in
`build/.config` with `configs/sim_defconfig` on top of it. Hardware-only
options, such as decks, sensors and motor drivers, are dropped. Everything
else carries over, for example the controller. The one exception is the
estimator: the Simmyflie always uses the Kalman estimator.

The Simmyflie is built with the host compiler. The ARM toolchain is not
needed.

Settings for the simulation itself are in `make menuconfig` under
**Simulation**. They take effect the next time you run `make sim`:

* Mass of the Crazyflie, and whether it has propeller guards
* Crash speed limit and crash angle limit
* The default start pose

## Running a Simmyflie

```sh
./build/sim.elf                  # listens on UDP port 50000
./build/sim.elf --port 50001     # pick another port
```

Options:

| Option                   | Description                                                   | Default             |
|--------------------------|---------------------------------------------------------------|---------------------|
| `--port <n>`, `-p <n>`   | UDP port to listen on                                         | 50000               |
| `--start-x <m>`          | Start position x, in m                                        | from `menuconfig`   |
| `--start-y <m>`          | Start position y, in m                                        | from `menuconfig`   |
| `--start-yaw <deg>`      | Start yaw, in degrees                                         | from `menuconfig`   |
| `--ground-level <m>`     | Ground level, in m                                            | 0                   |
| `--beacon <group:port>`  | Where to send beacons: an IPv4 multicast group and a UDP port | `239.255.50.0:49999`|
| `--help`                 | Print the options and exit                                    |                     |

Options with a value can also be written as `--name=value`, for example
`--start-x=-1.5`.

The firmware's console output is printed in the terminal. Stop the Simmyflie
with Ctrl+C.

To run several Simmyflies, start one per port, or use
[Simfield](#simfield).

## Connecting with cflib and cfclient

Use the URI `udp://localhost:<port>`, for example `udp://localhost:50000`,
anywhere you would use a `radio://` URI. For example, a
[cflib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/)
script:

```python
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

cflib.crtp.init_drivers()
with SyncCrazyflie('udp://localhost:50000', cf=Crazyflie()) as scf:
    print('Connected to', scf.cf.link_uri)
```

In cfclient, press **Scan**. Simmyflies listening on ports 50000 to 50500
show up as `udp://127.0.0.1:<port>`.

### Simulation parameters and logs

In addition to the normal firmware parameters and log variables, a
Simmyflie has:

* The `sim` parameter group: `sim.startX`, `sim.startY` (m) and
  `sim.startYaw` (degrees). Writing one moves the Simmyflie to the new start
  pose instantly, like carrying a Crazyflie to a new spot, and leaves it at
  rest. This works while it is at rest or crashed, and is refused while it is
  flying. This is also how you recover a crashed Simmyflie. The parameters
  hold the last written start pose, not where the Simmyflie currently is.
* The `trueState` log group: position (`x`, `y`, `z`), velocity (`vx`, `vy`,
  `vz`), acceleration (`ax`, `ay`, `az`), attitude quaternion (`qw`, `qx`,
  `qy`, `qz`), angular rate (`rateX`, `rateY`, `rateZ`), rotor speeds
  (`rotor1` to `rotor4`) and `status` (0 = at rest, 1 = flying,
  2 = crashed). All values are in SI units. Compare with `stateEstimate` to
  see how well the estimator is doing.

## Physics models

The Simmyflie at the moment only carries one physics model. This could change.
It is described below.

### Built-in CF2.1 Brushless

The built-in physics engine is the only one, and it is used whatever
platform the firmware is configured for. It is the Crazyflie 2.1 Brushless
model from A. Graefe, C. Scherer, W. Hönig and S. Trimpe, "How to Model Your
Crazyflie Brushless" (2026). The physics is stepped once per firmware tick,
at 1 kHz.

What is modelled:

* A rigid body with mass and inertia, without or with propeller guards. The
  mass can be changed in `menuconfig`; the inertia stays the one from the
  paper.
* The motors: each rotor speed follows the motor command with a 50 ms time
  constant.
* The thrust and drag torque of each rotor, as polynomials of the rotor speed
  fitted in the paper, and the reaction torque from accelerating a rotor.

Limitations:

* No aerodynamic effects beyond the rotor thrust and torque: no body drag,
  no ground effect, no downwash and no wind.
* No battery. The rotor speed depends only on the motor command, not on the
  battery voltage, and the battery voltage is always 4.2 V.
* No motor or propeller differences: all four are identical and ideal.
* The ground is the only thing a Simmyflie can hit. There are no walls or
  obstacles, and Simmyflies fly through each other.
* The ground contact is simple. Reaching ground level slower than the crash
  limits is a landing: the Simmyflie stops at once and is set level, keeping
  its yaw. Otherwise it is a crash and it freezes. It cannot slide, bounce or
  tip over.

## Beacons

Every Simmyflie sends a beacon 50 times per second (in sim time) with its true
pose, rotor speeds and status. Beacons let a program follow all Simmyflies on
the computer without connecting to any of them, so they don't take up a CRTP
connection. Nobody answers a beacon, and a Simmyflie doesn't know if anyone
is listening.

Beacons are UDP datagrams sent to an IPv4 multicast group, by default
`239.255.50.0:49999`. Change it with the Simmyflie's `--beacon` option. They
are sent on the loopback interface with a TTL of 0, so they never leave the
computer. Any number of programs can listen at the same time. Each one joins
the multicast group on `127.0.0.1`, with `SO_REUSEADDR` set so they can share
the port.

### Format

A beacon is 52 bytes, little-endian, with no padding. In Python `struct`
notation this is `"<BBHI11f"`.

| Offset | Type | Field            | Unit / values                                  |
|--------|------|------------------|------------------------------------------------|
| 0      | u8   | version          | 1                                              |
| 1      | u8   | status           | 0 = at rest, 1 = flying, 2 = crashed           |
| 2      | u16  | UDP port         | Port of the Simmyflie, identifies the instance |
| 4      | u32  | sim time         | ms, the firmware tick count                    |
| 8      | f32  | x, y, z          | m, world frame                                 |
| 20     | f32  | qw, qx, qy, qz   | Attitude quaternion, body to world             |
| 36     | f32  | rotor speed 1–4  | rad/s, M1 first                                |

The values are the same as in the `trueState` log group, and the pose,
status and sim time in a beacon all come from the same simulation step.

When you write your own listener:
* Accept a datagram only if its version is one you know and its length
  matches that version.
* Treat an unknown status value as unknown rather than dropping the beacon.
* Beacons can be lost. If one is missing, the next one comes as usual.

`tools/sim/simviewer.py` is the reference implementation of a listener.

## Simviewer

The simviewer shows all Simmyflies running on the computer in one 3D view.
It only listens to beacons that are sent to the multicast group and port as defined when starting the simviewer. The simviewer is entirely passive; it never connects to a Simmyflie, so cfclient or
your own scripts can stay connected while you watch.

The simviewer is a Python script with its dependencies declared inside, so you can run it with
[uv](https://docs.astral.sh/uv/):

```sh
uv run tools/sim/simviewer.py
```

Option:

* `--beacon <group:port>`: the beacon address to listen to. Default
  `239.255.50.0:49999`. It must match the Simmyflies' `--beacon` to pick up their beacons.

In the view:

* Each Simmyflie is drawn as an X-frame with a marker at the front, labelled
  with its UDP port, with a shadow on the ground.
* The colour shows the status: grey at rest, green flying, red crashed.
* A Simmyflie that has not sent a beacon for 1 s is drawn faded. After 10 s
  it is removed.
* The ground has a grid with the x axis (red) and y axis (green). Human
  figures, 1.8 m tall, every 5 m give a sense of scale.
* Drag with the mouse to rotate the view and use the scroll wheel to zoom.

Several simviewers can run at the same time, and each one shows every
Simmyflie.

## Simfield

Simfield starts a whole swarm of Simmyflies from one YAML file
and stops them all when you stop it.

```sh
uv run tools/sim/simfield.py swarm.yaml
```

Options:

* `--binary <path>`: the Simmyflie to run. Default `build/sim.elf` in the
  firmware repository Simfield is in.
* `--console`: print the console output of every Simmyflie, each line
  prefixed with the unit's name, or its port if it has no name. Without it
  the console output is not shown.

Simfield runs until you stop it with Ctrl+C. Then it stops every
Simmyflie it started.

### Swarm file

```yaml
name: Sim swarm                         # required
description: Two Simmyflies side by side
ground_level: 0.0                       # m, for every unit
beacon: 239.255.50.0:49999              # for every unit
units:                                  # required
- uri: udp://localhost:50000            # required
  name: sim-01
  start_x: 0.0                          # m
  start_y: 0.0                          # m
  start_yaw: 0                          # degrees
- uri: udp://localhost:50001
  name: sim-02
  start_x: 1.0
  start_y: 0.0
  start_yaw: 90
```

| Field           | Level | Required | Description                                   |
|-----------------|-------|----------|-----------------------------------------------|
| `name`          | swarm | yes      | Name of the swarm                             |
| `description`   | swarm | no       | Description of the swarm                      |
| `ground_level`  | swarm | no       | Ground level of every Simmyflie, in m         |
| `beacon`        | swarm | no       | Beacon address of every Simmyflie, `<multicast group>:<port>` |
| `units`         | swarm | yes      | List of units                                 |
| `uri`           | unit  | yes      | URI of the unit, e.g. `udp://localhost:50000` |
| `name`          | unit  | no       | Name of the unit                              |
| `description`   | unit  | no       | Description of the unit                       |
| `start_x`       | unit  | no       | Start position x, in m                        |
| `start_y`       | unit  | no       | Start position y, in m                        |
| `start_yaw`     | unit  | no       | Start yaw, in degrees                         |

* A field that is left out gets the Simmyflie's default.
* Fields Simfield does not know are ignored.
* One Simmyflie is started for each unit with the URI
  `udp://localhost:<port>` or `udp://127.0.0.1:<port>`, on that port. Other
  units, for example `radio://` ones, are skipped with a warning. This means
  the same file can describe a swarm that mixes real Crazyflies and
  Simmyflies.

## Typical sessions

### One Simmyflie

```sh
# Terminal 1: build and start a Simmyflie on port 50000
make cf21bl_defconfig
make sim
./build/sim.elf

# Terminal 2: watch it
uv run tools/sim/simviewer.py

# Terminal 3: fly it with cfclient or a cflib script, using
# udp://localhost:50000
```

### A swarm, with Simfield

```sh
# Terminal 1: build and start two Simmyflies
make cf21bl_defconfig
make sim
uv run tools/sim/simfield.py swarm.yaml --console

# Terminal 2: watch them
uv run tools/sim/simviewer.py

# Terminal 3: fly them with a cflib script
```

## Limitations

The Simmyflie has some limitations compared to the real Crazyflie:

- There are no decks. Deck drivers are not built, and every `deck.bc*`
  parameter reads 0.
- Out-of-tree apps cannot be built into the Simmyflie.
- The physics engine always uses Crazyflie Brushless dynamics, also when the
  firmware is configured for another platform, for example with
  `cf2_defconfig`.
- Persistent parameters are not stored. Storing one reports success, but
  nothing is saved, and the parameter is back at its default after a
  restart.
- The Simmyflie runs in real time, on the computer's clock. It cannot run
  faster or slower than real time, and a heavily loaded computer can make
  its timing uneven.

