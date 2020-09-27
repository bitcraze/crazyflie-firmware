---
title: Stabilizer Module
page_id: stabilizer_index
---

This page is meant as an introduction and overview of the path from
sensor acquisition to motor control,also called the stabilizer module. It will not go into detail but it mostly give a general outline of how the sensor measurements go to the
state estimators to the controllers and finally distributed to the motors
by power distribution. Ofcourse, the motors have an affect on how the
crazyflie flies and that inderectly has an effect on what the sensors
detect in the next time step.

 * [Sensors](#sensors)
 * [State Estimation](#state-estimation)
 * [State Controller](#state-controller)
 * [Configure estimators and control](#configuring-controllers-and-estimators)
 * [Commander Framework](#commander-framework)
 * [Power Distribution](#power-distribution)



### Overview

![sensor](/docs/images/sensors_to_motors.png){:width="700"}

## Modules


### Sensors

Sensors are essential for the flight of a crazyflie. Here is selection of the sensors
 listed that the crazyflie eventually uses for state estimation:


* [On-board Sensors](https://store.bitcraze.io/products/crazyflie-2-1)
  * Accelerometer: acceleration in body fixed coordinates in m/s2
  * Gyroscope: angle rate in roll pitch and yaw (rad/s)
  * Pressure Sensor: Airpressure in mBar
* [Flowdeck v2](https://store.bitcraze.io/products/flow-deck-v2)
  * ToF sensor*:  Distance to a surface in milimeters
  * Optical flow sensor:  The detection movement of pixels in px per timesample
* [Loco positioning deck](https://store.bitcraze.io//products/loco-positioning-deck):
  * Ultra Wide band module: The distance between two UWB modules or TDOA*** in meters.
* [Lighthouse deck](https://store.bitcraze.io/products/lighthouse-positioning-deck):
  * IR receivers: Sweep angle of htc vive basestations in radians.

<sub><sup>_*Time-of-Flight_</sup></sub>

<sub><sup>_**[Zranger v2](https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) also contains a laser-ranger_</sup></sub>

<sub><sup>_***Time-difference of Arrival_</sup></sub>

[go back to top](#)

### State Estimation

There are 2 state estimators in the crazyflie:
* Complementary Filter
* Extended Kalman Filter

 Go to the [state estimation page](state_estimators.md) for more indepth information about how the state estimation is implemented in the crazyflie firmware.

[go back to top](#)


### State Controller
There are 3 controllers in the crazyflie
* PID controller
* INDI controller
* Mellinger controller

Go to the [controllers page](controllers.md), for more indepth information about how the controllers are implemented in the crazyflie firmware.

[go back to top](#)


### Configuring Controllers and Estimators
Go to this [configuration page](configure_estimator_controller.md), if you would like to configure different controllers and estmators,

[go back to top](#)


### Commander Framework
An desired state can be handled by the setpoint structure in position or atitude, which can be set by the cflib or the highlevel commander.

Go to the [commander page](commanders_setpoints.md), for more indepth information about how the commander framework are implemented in the crazyflie firmware, please go

[go back to top](#)

### Power Distribution

After the state controller has send out its commands, this is not the end of the line yet.
The controllers send out their commands relating to their yaw, roll and pitch angles.
How the motors should respond in order to adhere these attitude based commands depends on a few factors:
  * Quadrotor configuration (found in: [power_distribution_stock.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/power_distribution_stock.c)):
    * x-configuration: The body fixed coordinate system's x-axis is pointed in between two propellors (Default)
    * +-configuration: The body fixed coordinate system's x-axis is pointed in one propellor
  * Motors:
    * **Explaination about this will come soon**


[go back to top](#)
