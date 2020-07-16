---
title: The Commander Framework
page_id: commanders_setpoints
---
This section will go into the commander framework, which handles the setpoint of the desired states, which the controllers will try to steer the estimated state to.

The Commander Module
=============
![commander framework](/images/commander_framework.png){:width="700"}

The commander module handles the incoming setpoints from several sources (src/modules/src/commander.c in the [firmware](https://github.com/bitcraze/crazyflie-firmware)). A setpoint can be set directly, either through a python script using the [cflib](https://github.com/bitcraze/crazyflie-lib-python)/ [cfclient](https://github.com/bitcraze/crazyflie-clients-python) or [the app layer](/building-and-flashing/build_instructions/#out-of-tree-build) (blue pathways in the figure), or by the high-level commander module (purple pathway). The High-level commander in turn, can be controlled remotely from the python library or from inside the Crazyflie.

It is important to realize that the commander module also checks how long ago a setpoint has been received. If it has been a little while (defined by threshold `COMMANDER_WDT_TIMEOUT_STABILIZE` in commander.c), it will set the attitude angles to 0 on order to keep the Crazyflie stabilized. If this takes longer than `COMMANDER_WDT_TIMEOUT_SHUTDOWN`, a null setpoint will be given which will result in the Crazyflie shutting down its motors and fall from the sky. This won’t happen if you are using the high level commander.

Setpoint Structure
=============

In order to understand the commander module, you must be able to comprehend the setpoint structure. The specific implementation can be found in src/modules/interface/stabilizer_types.h as setpoint_t in the Crazyflie firmware.

There are 2 levels to control, which is:

    Position (X, Y, Z)
    Attitude (pitch, roll, yaw or in quaternions)

These can be controlled in different modes, namely:

    Absolute mode (modeAbs)
    Velocity mode (modeVelocity)
    Disabled (modeDisable)

![commander framework](/images/setpoint_structure.png){:width="700"}


So if absolute position control is desired (go to point (1,0,1) in x,y,z), the controller will obey values given setpoint.position.xyz if setpoint.mode.xyz is set to modeAbs. If you rather want to control velocity (go 0.5 m/s in the x-direction), the controller will listen to the values given in setpoint.velocity.xyz if setpoint.mode.xyz is set to modeVel. All the attitude setpoint modes will be set then to disabled (modeDisabled). If only the attitude should be controlled, then all the position modes are set to modeDisabled. This happens for instance when you are controlling the crazyflie with a controller through the cfclient in attitude mode.

High Level Commander
=============

![high level commander](/images/high_level_commander.png){:width="700"}

As already explained before: The high level commander handles the setpoints from within the firmware based on a predefined trajectory. This was merged as part of the [Crazyswarm](https://crazyswarm.readthedocs.io/en/latest/) project of the [USC ACT lab](https://act.usc.edu/). The high-level commander uses a planner to generate smooth trajectories based on actions like ‘take off’, ‘go to’ or ‘land’ with 7th order polynomials. The planner generates a group of setpoints, which will be handled by the High level commander and send one by one to the commander framework.


Support in the python lib (CFLib)
=============

There are four main ways to interact with the commander framework from the [python library](https://github.com/bitcraze/crazyflie-lib-python)/.

* Send setpoints directly using the Commander class from the Crazyflie object, this can be seen in the autonomousSequence.py example for instance.
* Use the MotionCommander class, as in motion_commander_demo.py. The MotionCommander class exposes a simplified API and sends velocity setpoints continuously based on the methods called.
* Use the high level commander directly using the HighLevelCommander class on the Crazyflie object, see autonomous_sequence_high_level.py.
* Use the PositionHlCommander class for a simplified API to send commands to the high level commander, see the position_commander_demo.py
