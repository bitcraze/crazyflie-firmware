---
title: State estimation
page_id: state_estimators
---

A state estimator turns sensor signals into an estimate of the state that the crazyflie is in. This is an essential part of crazyflie's stabilizing system, as explained in the [overview page](/functional-areas/sensor_to_control/). State estimation is really important in quadrotors (and robotics in general). The Crazyflie needs to first of all know in which angles it is at (roll, pitch, yaw). If it would be flying at a few degrees slanted in roll, the crazyflie would accelerate into that direction. Therefore the controller need to know an good estimate of current angles’ state and compensate for it. For a step higher in autonomy, a good position estimate becomes important too, since you would like it to move reliably from A to B.

## Complementary filter

![complementary filter](/images/complementary_filter.png){:width="500"}

The complementary filter is consider a very lightweight and efficient filter which in general only uses the IMU input of the gyroscope (angle rate) and the accelerator. The estimator has been extended to also include input of the ToF distance measurement of the [Zranger deck](https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2). The estimated output is the Crazyflie’s attitude (roll, pitch, yaw) and its altitude (in the z direction). These values can be used by the controller and are meant to be used for manual control. 

To checkout the implementation details, please checkout the firmware in [estimator_complementary.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/estimator_complementary.c) and [sensfusion6.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/sensfusion6.c). The complementary filter is set as the default state estimator on the Crazyflie firmware, unless a deck is mounted that requires the kalman filter.

## Extended Kalman filter

![extended kalman filter](/images/extended_kalman_filter.png){:width="500"}

The (extended) Kalman filter (EKF) is an step up in complexity compared to the complementary filter, as it accepts more sensor inputs of both internal and external sensors. It is an recursive filter that estimates the current state of the Crazyflie based on incoming measurements (in combination with a predicted standard deviation of the noise), the measurement model and the model of the system itself. 

We will not go into detail on this but we encourage people to learn more about EKFs by reading up some material like [this](https://idsc.ethz.ch/education/lectures/recursive-estimation.html).

Because of more state estimation possibilities, we prefer the EKF for certain decks that can provide information for full pose estimation (position/velocity + attitude). These are the [Flowdeck v2](https://store.bitcraze.io/collections/decks/products/flow-deck-v2), [Loco positioning deck](https://store.bitcraze.io/collections/positioning/products/loco-positioning-deck) and the [lighthouse deck](https://store.bitcraze.io/products/lighthouse-positioning-deck). When the `DeckDriver` is initialized in the [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware/), for these decks the variable `.requiredEstimator` is set to `kalmanEstimator`. This says that the firmware should not use the default complementary filter but the EKF instead.

Here we will explain a couple of important elements that are essential to the implementation, however we do encourage people to also look into the code [estimator_kalman.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/estimator_kalman.c) and [kalman_core.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/kalman_core.c). Also read the papers of [1] and [2] for implementation details.

### Kalman supervisor

The Kalman filter has an supervisor that resets the state estimation if the values get out of bounds. This can be found here in [kalman_supervisor.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/kalman_supervisor.c). 

### Measurement Models

This section will explain how the signals of the sensors are transformed to state estimates. These equations are the base of the measurement models of the EKF.


#### Flowdeck

This illustration explains how the height from the VL53L1x sensor and flow from the PMW3901 sensor are combined to calculate velocity. This has been implemented by the work of [3] and can be found in [kalman_core.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/kalman_core.c) in the function `kalmanCoreUpdateWithFlow()`.

![flowdeck velocity](/images/flowdeck_velocity.png){:width="500"}

#### Locodeck

COMING SOON!

#### Lighthouse Measurement Model

COMING SOON!

### References for implementation details
[1] Mueller, Mark W., Michael Hamer, and Raffaello D'Andrea. "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation." 2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015.

[2] Mueller, Mark W., Markus Hehn, and Raffaello D’Andrea. "Covariance correction step for kalman filtering with an attitude." Journal of Guidance, Control, and Dynamics 40.9 (2017): 2301-2306.

[3] M. Greiff, Modelling and Control of the Crazyflie Quadrotor for Aggressive and Autonomous Flight by Optical Flow Driven State Estimation, Master’s thesis, Lund University, 2017



