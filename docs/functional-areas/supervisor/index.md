---
title: The supervisor
page_id: supervisor_index
---

The purpose of the supervisor is to monitor the system and its state. Depending on the situation, the supervisor
can enable/disable functionality as well as take action to protect the system or humans close by.

The supervisor is based on a state machine that is driven from the supervisor loop. It is given the opportunity
to modify other modules and data, such as the High level commander, setpoints, and motor control, to handle exceptional
situations or for protection.

## The update sequence

The update sequence is separated into a few steps:
1. Collect data from sensors and the system. We call these [conditions](conditions.md).
2. Based on the conditions, check if the state machine should [transition](transitions.md) into a new [state](states.md).
3. If there is a state [transition](transitions.md), possibly execute one ore more actions.
4. Set the new state.

## Modifying behavior

One modification of behavior is to prevent the motors from spinning when the system is not ready to fly, for
instance if the system is not armed or the Crazyflie is up side down. Also the high level commander is blocked from
running trajectories in this case.

Another modification handles exceptional situations when flying, for instance when tumbling is detected. This results in a crashed state and is simply handled by stopping the
motors and free falling.

The supervisor framework provides the possibility to handle situations in a more "clever" way, such as doing a controlled
landing when possible, instead of free falling, but that is currently not implemented.

## Emergency stops

The supervisor supports two emergency stop mechanisms: an immediate emergency stop command and an emergency stop watchdog.

The immediate emergency stop command stops the motors as soon as the command is handled by the supervisor. It puts the supervisor in the locked state, which is latching and requires a reboot to recover.

The emergency stop watchdog can be used to implement an emergency stop command with a periodic keepalive. It is disabled at startup and becomes active when the first watchdog keepalive packet is received. After that, a new keepalive must be received at least once every 1000 ms. If the source stops sending keepalive packets and the timeout expires, the Crazyflie enters emergency stop, stops the motors and enters the locked state.

The watchdog mechanism can be used by external control or safety integrations that want the Crazyflie to default to emergency stop if communication is lost. For packet-level details, see the [Supervisor CRTP port](/docs/functional-areas/crtp/crtp_supervisor.md).

## Sub pages

{% sub_page_menu %}
