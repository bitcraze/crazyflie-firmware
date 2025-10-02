---
title: Crash Handling
page_id: supervisor_crash_handling
sort_order: 5
---

A crash can occur when the Crazyflie is subject to exceptional situations when flying. The crash handling in the Crazyflie consists of a `crashed` state, as well as a `crashed` condition that has to be fulfilled to enter the crashed state, both of which are implemented in the supervisor. 

The only way for the Crazyflie to get into a `crashed` state is by being `tumbled` when flying. When entering the crashed state, the default action for the Crazyflie is to cut the thrust to all motors and free fall to avoid running the propellers when accidentally hitting, for example, the walls or ground. However, this handling can be updated to other methods.

## Tumbled state

The Crazyflie can enter the `tumbled` state in two ways:
1. The Crazyflie is tilted at a large angle for a long period of time.
2. The Crazyflie is close to, or completely, upside down for a short period of time.
The details, such as the time and angle required to tumble the Crazyflie, are specified in the supervisor. 

To verify that the Crazyflie is no longer tumbled it can be put upright and level on the ground.

## Crash recovery request

The Crazyflie can get out of the crashed state through a crash recovery request, `supervisorRequestCrashRecovery(const bool doRecover);`, which is a message that is sent to the supervisor. 

This request will only result in a recovery, i.e. get the Crazyflie out of the crashed state, if the Crazyflie is no longer in a tumbled state. 