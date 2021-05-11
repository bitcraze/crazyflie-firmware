---
title: The Loco Positioning System
page_id: loco_positioning
---

The Loco Positioning System is an absolute positioning system based on Ultra Wide Band radios.

Most of the documentation for the Loco Positioning System can be found in the [lps-node-firmware repository documentation](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/), this is where protocols, principles and modes are described.

This section contains details specific to the implementation in the Crazyflie, mainly estimator related information.

## Position estimation

When using the Loco Positioning System the [Kalman estimator](/docs/functional-areas/sensor-to-control/state_estimators.md#extended-kalman-filter) is used for position estimation.

TODO: document measurement models for TWR, TWR robust, TDoA and TDoA robust.
