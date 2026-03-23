---
title: The Commander Framework
page_id: commanders_setpoints
---

This section describes the commander module, which acts as a **setpoint multiplexer**: it receives setpoints from several sources and selects the active one based on priority.

 * [The Commander Module](#the-commander-module)
 * [Setpoint Structure](#setpoint-structure)
 * [Setpoint Priority](#setpoint-priority)


## The Commander Module

![commander framework](/docs/images/commander_framework.png){:width="700"}

The commander module handles the incoming setpoints from several sources (src/modules/src/commander.c in the [firmware](https://github.com/bitcraze/crazyflie-firmware)). Setpoints can arrive from:

* **CRTP packets**: sent from a ground station using the [cflib](https://github.com/bitcraze/crazyflie-lib-python) / [cfclient](https://github.com/bitcraze/crazyflie-clients-python). This includes gamepad input when routed through cfclient.
* **External RC receiver (EXTRX)**: from a physical RC receiver (CPPM/Spektrum) connected directly to the Crazyflie via a deck.
* **Onboard guidance**: generated autonomously onboard by the built-in [High-Level Commander](onboard_guidance.md) or an out-of-tree implementation.
* **App layer**: from code running on the Crazyflie itself via [the app layer](/docs/userguides/app_layer.md).

Not all sources need to be active at the same time; the commander simply picks the highest-priority setpoint it has received.

It is important to realize that the commander module also checks how long ago a setpoint has been received. If it has been a little while (defined by threshold `COMMANDER_WDT_TIMEOUT_STABILIZE` in commander.c), it will set the attitude angles to 0 in order to keep the Crazyflie stabilized. If this takes longer than `COMMANDER_WDT_TIMEOUT_SHUTDOWN`, a null setpoint will be given which will result in the Crazyflie shutting down its motors and falling from the sky. This won't happen if you are using onboard guidance, as it continuously feeds setpoints to the commander.

## Setpoint Structure


In order to understand the commander module, you must be able to comprehend the setpoint structure. The specific implementation can be found in src/modules/interface/stabilizer_types.h as setpoint_t in the Crazyflie firmware.

There are 2 levels to control, which is:

    Position (X, Y, Z)
    Attitude (pitch, roll, yaw or in quaternions)

These can be controlled in different modes, namely:

    Absolute mode (modeAbs)
    Velocity mode (modeVelocity)
    Disabled (modeDisable)

![setpoint structure](/docs/images/setpoint_structure.png){:width="700"}


So if absolute position control is desired (go to point (1,0,1) in x,y,z), the controller will obey values given setpoint.position.xyz if setpoint.mode.xyz is set to modeAbs. If you rather want to control velocity (go 0.5 m/s in the x-direction), the controller will listen to the values given in setpoint.velocity.xyz if setpoint.mode.xyz is set to modeVel. All the attitude setpoint modes will be set then to disabled (modeDisabled). If only the attitude should be controlled, then all the position modes are set to modeDisabled. This happens for instance when you are controlling the Crazyflie with a controller through the [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) in attitude mode.


## Setpoint Priority

The commander framework may receive setpoints from multiple sources simultaneously.
The priority hierarchy from lowest to highest is:

1. **DISABLE**: no setpoints.
2. **ONBOARD_GUIDANCE**: setpoints from the High-Level Commander or an OOT onboard guidance module.
3. **CRTP**: setpoints sent from a ground station via CRTP.
4. **EXTRX**: setpoints from a physical RC receiver connected to the Crazyflie.

CRTP and EXTRX setpoints have higher priority than onboard guidance, so a user can always
take over control to handle an emergency situation. When a higher-priority source takes over,
the active onboard guidance is stopped. It will not generate any more setpoints until it is
explicitly re-enabled by a call to the `commanderRelaxPriority()` function
(or `cf.commander.send_notify_setpoint_stop()` from the python lib).

Note that the [supervisor](/docs/functional-areas/supervisor/) will check that there is a continuous stream of setpoints
received by the commander framework as long as the platform is flying, if not it will take action to protect the
platform and people around it, possibly ending up in a locked state.

## Support in the python lib (CFLib)

There are two main ways to send low level setpoints from the [python library](https://github.com/bitcraze/crazyflie-lib-python):

* **autonomousSequence.py**: Send setpoints directly using the Commander class from the Crazyflie object.
* **motion_commander_demo.py**: The MotionCommander class exposes a simplified API and sends velocity setpoints continuously based on the methods called.

For examples that use onboard guidance (High-Level Commander), see the [Onboard Guidance](onboard_guidance.md) page.
