---
title: Arming
page_id: supervisor_arming
sort_order: 4
---

Arming is a manual action required before take off to let the system know that the pilot is in control of the system
and is ready to fly. Arming is implemented as a condition in the supervisor and is required to transition from the
`Pre flight checks passed` state to `Ready to fly`. The condition is satisfied by making an arming request in the
supervisor, that is arming in an event. If the supervisor state transitions back to `Pre flight checks passed`, the
system must be armed again to be able to fly.

## Arming request

Clients perform an arming request by sending a [CRTP arming message](/docs/functional-areas/crtp/crtp_platform.md#armdisarm-system)
to the Crazyflie.

Apps running in the Crazyflie request arming by calling the `supervisorRequestArming()` function.

## Auto arming

On small, safe platforms with brushed motors, like the Crazyflie 2.X, it is possible to configure auto arming. When
auto arming is enabled, the system will automatically request arming when entering the `Pre flight checks passed`
state. This will (if all other conditions are met) make the supervisor transition into the `Ready to fly` state.

Auto arming is the default setting for the Crazyflie 2.X.

Auto arming is configured at compile time through the `CONFIG_MOTORS_REQUIRE_ARMING=y` kconfig flag.

## Idle thrust

The motors are used to indicate to the pilot that the system is armed and ready to fly. Motors runs at idle thrust when
the supervisor is in a state where flight is enabled.

Note that the default
settings for idle thrust on the Crazyflie 2.X is 0 and the motors will not spin. On platforms using brushless motors,
an idle thrust that spins the motors should be used.

Idle thrust is configured at compile time through the `CONFIG_MOTORS_DEFAULT_IDLE_THRUST` kconfig parameter.
