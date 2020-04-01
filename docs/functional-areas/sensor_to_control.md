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

Sensors
==================
Sensors are essential for the flight of a crazyflie. Here is a table with the sensors
 listed that the crazyflie eventually uses for state estimation:

| Location | Type | Measurement | Unit | Sensor | driver |
| -------- | ----- | ---------- | ----- | ----- | ---- |
| On-board Sensors | Accelerometer | acceleration in body fixed coordinates | m/s2 | BMI088 | [sensors_bmi088_bmp388.c](https://github.com/bitcraze/crazyflie-firmware/src/hal/src/sensors_bmi088_bmp388.c) |
| " "  | Gyroscope | angle rate in roll pitch and yaw | rad/s | " " | " " |
|  " "  | Pressure Sensor | Airpressure | mBar | BMP388 | " "|
| Z-ranger v2 | ToF sensor* | Distance to a surface | milimeters | VL53L1x | [vl53l1x.c](https://github.com/bitcraze/crazyflie-firmware/src/drivers/src/vl53l1x.c)
| Flowdeck v2** | Optical flow sensor | The detection movement of pixels | px per timesample |PMW3901 | [pmw3901.c](https://github.com/bitcraze/crazyflie-firmware/src/drivers/src/pmw3901.c)
| LPS deck | Ultra Wide band | The Distance between two UWB modules or TDOA*** |meters|DWM1000| [locodeck.c](https://github.com/src/deck/drivers/src/locodeck.c)|
| Lighthouse deck | IR receivers | Sweep angle of htc vive basestations | rad | TS4231 |  [lighthouse.c](https://github.com/src/src/deck/drivers/src/lighthouse.c)|

*Time-of-Flight

**Also contains a laser-ranger

***Time-difference of Arrival

State Estimation
=============
COMING SOON!

State Controller
==========================
COMING SOON!


Power Distribution
=============
COMING SOON!

