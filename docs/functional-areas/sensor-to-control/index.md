---
title: Sensor to control
page_id: sensor_to_control
---

This page is meant as an introduction and overview of the path from
sensor acquisition to motor control. It will not go into detail but it
mostly give a general outline of how the sensor measurements go to the 
state estimators to the controllers and finally distributed to the motors
by power distribution. Ofcourse, the motors have an affect on how the 
crazyflie flies and that inderectly has an effect on what the sensors
detect in the next time step.

![sensor](/images/sensors_to_motors.png){:width="700"}

## Modules 

### Sensors
==================
Sensors are essential for the flight of a crazyflie. Here is a table with the sensors
 listed that the crazyflie eventually uses for state estimation:

| Location | Type | Measurement | Unit | Sensor | driver |
| -------- | ----- | ---------- | ----- | ----- | ---- |
| On-board Sensors | Accelerometer | acceleration in body fixed coordinates | m/s2 | BMI088 | [sensors_bmi088_bmp388.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/hal/src/sensors_bmi088_bmp388.c) |
| " "  | Gyroscope | angle rate in roll pitch and yaw | rad/s | " " | " " |
|  " "  | Pressure Sensor | Airpressure | mBar | BMP388 | " "|
| Z-ranger v2 | ToF sensor* | Distance to a surface | milimeters | VL53L1x | [vl53l1x.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/vl53l1x.c)
| Flowdeck v2** | Optical flow sensor | The detection movement of pixels | px per timesample |PMW3901 | [pmw3901.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/pmw3901.c)
| LPS deck | Ultra Wide band | The Distance between two UWB modules or TDOA*** |meters|DWM1000| [locodeck.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/locodeck.c)|
| Lighthouse deck | IR receivers | Sweep angle of htc vive basestations | rad | TS4231 |  [lighthouse.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/lighthouse.c)|

*Time-of-Flight

**Also contains a laser-ranger

***Time-difference of Arrival

### State Estimation

There are 2 state estimators in the crazyflie:
* Complementary Filter
* Extended Kalman Filter

For more indepth information about how the state estimation is implemented in the crazyflie firmware, please go to the [state estimation page](/functional-areas/state_estimators/)

### State Controller
There are 3 controllers in the crazyflie
* PID controller
* INDI controller
* Mellinger controller

For more indepth information about how the controllers are implemented in the crazyflie firmware, please go to the [controllers page](/functional-areas/controllers/)

### Commander Framework
An desired state can be handled by the setpoint structure in position or atitude, which can be set by the cflib or the highlevel commander

For more indepth information about how the commander framework are implemented in the crazyflie firmware, please go to the [commander page](/functional-areas/commanders_setpoints/)

### Power Distribution

After the state controller has send out its commands, this is not the end of the line yet.
The controllers send out their commands relating to their yaw, roll and pitch angles.
How the motors should respond in order to adhere these attitude based commands depends on a few factors:
  * Quadrotor configuration (found in: [power_distribution_stock.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/power_distribution_stock.c)): 
    * x-configuration: The body fixed coordinate system's x-axis is pointed in between two propellors (Default)
    * +-configuration: The body fixed coordinate system's x-axis is pointed in one propellor
  * Motors:
    * **Explaination about this will come soon**


