---
title: The Commander Framework
page_id: commanders_setpoints
---


This section will go into the commander framework, which handles the setpoint of the desired states, which the controllers will try to steer the estimated state to.

 * [The Commander Module](#the-commander-module)
 * [Setpoint Structure](#setpoint-structure)
 * [High Level Commander](#high-level-commander)


## The Commander Module

![commander framework](/docs/images/commander_framework.png){:width="700"}

The commander module handles the incoming setpoints from several sources (src/modules/src/commander.c in the [firmware](https://github.com/bitcraze/crazyflie-firmware)). A setpoint can be set directly, either through a python script using the [cflib](https://github.com/bitcraze/crazyflie-lib-python)/ [cfclient](https://github.com/bitcraze/crazyflie-clients-python) or [the app layer](/docs/userguides/app_layer.md) (blue pathways in the figure), or by the high-level commander module (purple pathway). The High-level commander in turn, can be controlled remotely from the python library or from inside the Crazyflie.

It is important to realize that the commander module also checks how long ago a setpoint has been received. If it has been a little while (defined by threshold `COMMANDER_WDT_TIMEOUT_STABILIZE` in commander.c), it will set the attitude angles to 0 on order to keep the Crazyflie stabilized. If this takes longer than `COMMANDER_WDT_TIMEOUT_SHUTDOWN`, a null set-point will be given which will result in the Crazyflie shutting down its motors and fall from the sky. This won’t happen if you are using the high level commander.

## Setpoint Structure


In order to understand the commander module, you must be able to comprehend the set-point structure. The specific implementation can be found in src/modules/interface/stabilizer_types.h as setpoint_t in the Crazyflie firmware.

There are 2 levels to control, which is:

    Position (X, Y, Z)
    Attitude (pitch, roll, yaw or in quaternions)

These can be controlled in different modes, namely:

    Absolute mode (modeAbs)
    Velocity mode (modeVelocity)
    Disabled (modeDisable)

![commander framework](/docs/images/setpoint_structure.png){:width="700"}


So if absolute position control is desired (go to point (1,0,1) in x,y,z), the controller will obey values given setpoint.position.xyz if setpoint.mode.xyz is set to modeAbs. If you rather want to control velocity (go 0.5 m/s in the x-direction), the controller will listen to the values given in setpoint.velocity.xyz if setpoint.mode.xyz is set to modeVel. All the attitude setpoint modes will be set then to disabled (modeDisabled). If only the attitude should be controlled, then all the position modes are set to modeDisabled. This happens for instance when you are controlling the Crazyflie with a controller through the [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) in attitude mode.


## High Level Commander

![high level commander](/docs/images/high_level_commander.png){:width="700"}

As already explained before: The high level commander generates setpoints from within the firmware based on a predefined trajectory. This was merged as part of the [Crazyswarm](https://crazyswarm.readthedocs.io/en/latest/) project of the [USC ACT lab](https://act.usc.edu/). The high-level commander uses a planner to generate smooth trajectories based on actions like ‘take off’, ‘go to’ or ‘land’ with 7th order polynomials. The planner generates a group of set-points, which will be handled by the High level commander and send one by one to the commander framework.


### Setpoint priority

The commander framework may receive setpoints from both the High level commander as well as low level setpoints,
for instance from a user on the ground. Low level setpoints will always have higher priority as they might originate
from a user that wants to take over control from the High level commander and handle an emergency situation.
Once the High level commander has been disabled by receiving a low level setpoint, it will not generate any more
setpoints again until it explicitly is enabled by a call to the `commanderRelaxPriority()` function
(or `cf.commander.send_notify_setpoint_stop()` from the python lib).

Note that the [supervisor](/docs/functional-areas/supervisor/) will check that there is a continuous stream of setpoints
received by the commander framework as long as the platform is flying, if not it will take action to protect the
platform and people around it, possibly ending up in a locked state.

### Switching between High level commander and low level setpoints

There might be a need to go back and forth between the High level commander and low level setpoints. Going from the
High level commander to low level setpoints is as easy as starting to send the low level setpoints. Going back to the
High level commander requires a call to the `commanderRelaxPriority()` function (or
`cf.commander.send_notify_setpoint_stop()` from the python lib) to enable the High level commander again.

Note that it takes a few seconds for the platform to understand that it is not flying after landing, and if you are using
a script or application that is feeding low level setpoints to the Crazyflie during the landing phase, you have to
continue to feed zero setpoints for a while to avoid that the supervisor locks the platform. Another option is to
re-enable the high level commander as it continuously is feeding zero setpoints to the commander framework, also when
not flying a trajectory.


## Support in the python lib (CFLib)

There are four main ways to interact with the commander framework from the [python library](https://github.com/bitcraze/crazyflie-lib-python)/.

* **autonomousSequence.py**: Send setpoints directly using the Commander class from the Crazyflie object.
* **motion_commander_demo.py**: The MotionCommander class exposes a simplified API and sends velocity set-points continuously based on the methods called.
* **autonomous_sequence_high_level.py**: Use the high level commander directly using the HighLevelCommander class on the Crazyflie object.
* **position_commander_demo.py**: Use the PositionHlCommander class for a simplified API to send commands to the high level commander.
