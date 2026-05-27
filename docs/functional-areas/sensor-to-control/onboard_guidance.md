---
title: Onboard Guidance
page_id: onboard_guidance
---

## Overview

Onboard guidance is the part of the system that generates setpoints autonomously onboard the Crazyflie. This is **optional**: setpoints can also come directly from a ground station via CRTP, or from the app layer, without any onboard guidance needed.

Onboard guidance is useful when the Crazyflie needs to decide what to do on its own: autonomous flight, trajectory following, neural-net navigation, and so on. The onboard guidance module receives the current state estimate as input, produces setpoints as output, and feeds them to the [commander](commanders_setpoints.md) at `COMMANDER_PRIORITY_ONBOARD_GUIDANCE`.

Like controllers and estimators, onboard guidance supports runtime switching via the `stabilizer.guidance` parameter.

## High-Level Commander

![high level commander](/docs/images/high_level_commander.png){:width="700"}

The High-Level Commander is the built-in onboard guidance implementation. It was merged as part of the [Crazyswarm](https://crazyswarm.readthedocs.io/en/latest/) project of the [USC ACT lab](https://act.usc.edu/). It uses a planner to generate smooth trajectories based on actions like *take off*, *go to*, or *land* with 7th order polynomials. The planner generates a group of setpoints, which are sent one by one to the commander framework.

The High-Level Commander can be controlled remotely via CRTP from the python library, or from inside the Crazyflie using the internal C API:

* `crtpCommanderHighLevelTakeoff()`
* `crtpCommanderHighLevelLand()`
* `crtpCommanderHighLevelGoTo()`
* `crtpCommanderHighLevelStop()`

See `src/modules/interface/crtp_commander_high_level.h` for the full API.

### Support in the python lib (CFLib)

The [python library](https://github.com/bitcraze/crazyflie-lib-python) provides two classes for interacting with the High-Level Commander:

* **HighLevelCommander**: direct access to the High-Level Commander (example: `autonomous_sequence_high_level.py`).
* **PositionHlCommander**: a simplified API that sends commands to the High-Level Commander (example: `position_commander_demo.py`).

For low level setpoint examples, see the [Commander Framework](commanders_setpoints.md) page.

## Switching between onboard guidance and other setpoint sources

Other setpoint sources can be [prioritized over](commanders_setpoints.md#setpoint-priority) onboard guidance.
Going from onboard guidance to direct setpoints is as easy as starting to send them. Going back to
onboard guidance requires a call to the `commanderRelaxPriority()` function (or
`cf.commander.send_notify_setpoint_stop()` from the python lib) to re-enable onboard guidance.

Note that it takes a few seconds for the platform to understand that it is not flying after landing, and if you are using
a script or application that is feeding setpoints to the Crazyflie during the landing phase, you have to
continue to feed zero setpoints for a while to avoid that the supervisor locks the platform. Another option is to
re-enable onboard guidance as it continuously feeds zero setpoints to the commander framework, also when
not flying a trajectory.

## Out-of-Tree Onboard Guidance

When `CONFIG_ONBOARD_GUIDANCE_OOT` is selected as the default, the OOT implementation is used at startup. The High-Level Commander can still be compiled alongside it (controlled by `CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE`), allowing OOT code to use the HLC as a library — for example, delegating trajectory planning while adding custom logic on top.

Seven functions must be implemented:

* `void onboardGuidanceOutOfTreeInit(void)`: called once at startup.
* `bool onboardGuidanceOutOfTreeTest(void)`: return `true` if initialization succeeded.
* `bool onboardGuidanceOutOfTreeGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep)`: called every stabilizer loop iteration. Write the desired setpoint and return `true` if a setpoint was produced.
* `void onboardGuidanceOutOfTreeStop(void)`: called when a higher-priority source takes over.
* `void onboardGuidanceOutOfTreeTellState(const state_t *state)`: called when onboard guidance is re-enabled (via `commanderRelaxPriority()`) after a higher-priority source, to provide a current state snapshot. It is not called periodically; `state` is already passed to `onboardGuidanceOutOfTreeGetSetpoint()` on every stabilizer loop.
* `void onboardGuidanceOutOfTreeBlock(bool doBlock)`: called every stabilizer loop iteration. When `doBlock` is `true`, the guidance must not produce setpoints (motors are not allowed to run). This is critical for safety.
* `bool onboardGuidanceOutOfTreeIsDone(void)`: return `true` if the guidance has finished its current task (e.g. trajectory completed).

These are declared in `src/modules/interface/onboard_guidance.h`.

See `examples/app_out_of_tree_onboard_guidance/` for a working example and the [OOT build documentation](/docs/development/oot.md) for the general build setup.
