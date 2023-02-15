---
title: Stabilizer Module
page_id: stabilizer_index
---

This page is meant as an introduction and overview of the path from
sensor acquisition to motor control, also called the stabilizer module. It will not go into detail but it mostly gives a general outline of how the sensor measurements go to the
state estimators to the controllers and finally distributed to the motors
by power distribution. Of course, the motors have an affect on how the
Crazyflie flies and that indirectly has an effect on what the sensors
detect in the next time step.

 * [Sensors](#sensors)
 * [State Estimation](state_estimators.md)
 * [State Controller](controllers.md)
 * [Commander Framework](commanders_setpoints.md)
 * [Power Distribution](#power-distribution)
 * [Configure estimators and control](configure_estimator_controller.md)



### Overview

![sensor](/docs/images/sensors_to_motors.png)

## Modules


### Sensors

Sensors are essential for the flight of a Crazyflie. Here is selection of the sensors
 listed that the Crazyflie eventually uses for state estimation:


* [On-board Sensors](https://store.bitcraze.io/products/crazyflie-2-1)
  * Accelerometer: acceleration in body fixed coordinates in m/s2
  * Gyroscope: angle rate in roll pitch and yaw (rad/s)
  * Pressure Sensor: Air pressure in mBar
* [Flowdeck v2](https://store.bitcraze.io/products/flow-deck-v2)
  * ToF sensor*:  Distance to a surface in mili-meters
  * Optical flow sensor:  The detection movement of pixels in px per timesample
* [Loco positioning deck](https://store.bitcraze.io//products/loco-positioning-deck):
  * Ultra Wide band module: The distance between two UWB modules or TDOA*** in meters.
* [Lighthouse deck](https://store.bitcraze.io/products/lighthouse-positioning-deck):
  * IR receivers: Sweep angle of htc vive base stations in radians.
* MoCap: [Active Marker deck](https://www.bitcraze.io/products/active-marker-deck/) or [Motion capture deck](https://www.bitcraze.io/products/motion-capture-marker-deck/)
  * External calculated position and orientation, usually broadcasted to the Crazyflie by Crazyradio.

<sub><sup>_*Time-of-Flight_</sup></sub>

<sub><sup>_**[Zranger v2](https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) also contains a laser-ranger_</sup></sub>

<sub><sup>_***Time-difference of Arrival_</sup></sub>


### State Estimation

There are 2 state estimators in the Crazyflie:
* Complementary Filter
* Extended Kalman Filter

 Go to the [state estimation page](state_estimators.md) for more in-depth information.


### State Controller
There are 3 controllers in the Crazyflie
* PID controller
* INDI controller
* Mellinger controller

Go to the [controllers page](controllers.md), for more in-depth information.


### Commander Framework
A desired state can be handled by the set-point structure in position or attitude, which can be set by the [cflib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/) or the High Level commander.

Go to the [Commander page](commanders_setpoints.md), for more in-depth information.

### Power Distribution

After the state controller has sent out its commands, this is not the end of the line yet.
The controllers send out their commands relating to their yaw, roll and pitch angles.
How the motors should respond in order to adhere these attitude based commands depends on a few factors:
  * Motors:
    * Brushed: The Crazyflie has brushed motors, of which there is battery compensation function enabled. Check out `motors.c` to learn more. Also checkout the [PWM to Thrust investigations](/docs/functional-areas/pwm-to-thrust.md) of those same motors.
    * Brushless: The Bolt enables the control of brushless motors. Checkout the[ product page of the Bolt](https://www.bitcraze.io/products/crazyflie-bolt/) for more information.

## Configuring Controllers and Estimators
Go to this [configuration page](configure_estimator_controller.md), if you would like to configure different controllers and estimators.
