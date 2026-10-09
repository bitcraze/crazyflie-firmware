---
title: Autonomous flight
page_id: autonomous_flight
---

## Take off

When flying autonomously, it's important to get a few things right before take-off. This applies both to apps running in the
Crazyflie and to remote clients such as cflib, cflib2 and crazyflie-lib-rs. These are the minimum steps required to get
off the ground, they are **not** a complete pre-flight checklist.

This page collects the steps required in the order they must happen.

### 1. Wait for the system to start

**Firmware apps**: The firmware calls `appMain()` once the system task has started, but this does not mean that the
Crazyflie is ready to fly. Estimators and decks may still be settling. Many of the examples start with a
`vTaskDelay(M2T(2000))`, which is only a simple way to let the system get going before printing or acting. It is not a
guarantee of anything, and for flight you should wait for the conditions below.

**Remote clients**: The radio link is enabled before the decks are initialized, so a client can technically connect
while the system is still booting. In practice, setting up the connection normally takes longer than the boot.

### 2. Wait for the position estimate to converge

If the Crazyflie takes off before the position estimate has converged, the position control acts on a bad estimate
and the Crazyflie may fly off or crash.

The [supervisor](/docs/functional-areas/supervisor/index.md) does **not** check estimator convergence. It is up to the app or client to wait for it.

In cflib, `reset_estimator()` in `cflib.utils.reset_estimator` resets the estimator and then waits for the variances to settle.

### 3. Arm the system

The supervisor must be [armed](/docs/functional-areas/supervisor/arming.md) before the motors are allowed to run. This is not needed if
[auto arming](/docs/functional-areas/supervisor/arming.md#auto-arming) is enabled. Auto arming is the default on the brushed Crazyflie 2.1(+).

Platforms that require arming, such as the Crazyflie 2.1 Brushless and Bolt, need an explicit arming request.

Apps arm the system by calling `supervisorRequestArming(true)`, remote clients by sending a CRTP
[arming message](/docs/functional-areas/crtp/crtp_supervisor.md#armdisarm-system). In cflib this is
`cf.supervisor.send_arming_request(True)`, see for instance `examples/step-by-step/sbs_motion_commander.py`.

### 4. Take off before the preflight timeout

Once armed, the Crazyflie must take off within the preflight timeout. If it does not, the supervisor leaves the
`Ready to fly` [state](/docs/functional-areas/supervisor/states.md), goes back to `Pre flight checks passed` and disarms. The system must then be
armed again.

The timeout is set by the `supervisor.prefltTimeout` parameter, in milliseconds. The default is 30000 (30 s), defined
by `PREFLIGHT_TIMEOUT_MS` in `platform_defaults.h`.

### Examples

Basic examples that follow these steps can be found in each repository:

- Apps running in the Crazyflie: the [examples folder](https://github.com/bitcraze/crazyflie-firmware/tree/master/examples) in the crazyflie-firmware repository.
- Python: the [examples folder](https://github.com/bitcraze/crazyflie-lib-python/tree/master/examples) in the crazyflie-lib-python (cflib) repository, and in the cflib2 repository: [examples folder](https://github.com/bitcraze/crazyflie-lib-python-v2/tree/main/examples).
- The crazyflie-lib-rs repository also has an [examples folder](https://github.com/bitcraze/crazyflie-lib-rs/tree/main/examples).

For more complete examples, see the [crazyflie-demos](https://github.com/bitcraze/crazyflie-demos) repository.
