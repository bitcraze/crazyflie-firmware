---
title: PWM to Thrust
page_id: pwm-to-thrust
---

# This is the analyses of finding a PWM to thrust transfer function
## Introduction
To do modeling, simulations or to improve the flight algorithms the physical parameters of the Crazyflie system is good to know. The following test are done to find some of these parameters.

## Measuring the RPM
To measure the RPM we used optical switches that we connected to a prototype board. Details can be found in this blog post. The firmware was then modified to measure the timing of the optical switches, convert them to RPM and make them available to the log subsystem. The log subsystem was also “hacked” so that more frequent samples could be taken (500Hz instead of max 100Hz)

![rpm board](/docs/images/rpm-board.jpg)

## Measuring the thrust
To measure the thrust we built a simple test fixture. We use a precision scale to measure the thrust. It is done by letting a weight hold the Crazyflie 2.X down that is standing the scale. The lift generated will show up on the scale as the total weight gets lighter. We glued a prototype expansion board to a bottle which we could attach the Crazyflie 2.X to. It is not a perfect fixture but shouldn't be too far off.

![cf2 thrust fixture](/docs/images/cf2_thrust_fixture.jpg)

## Measuring amps and voltage
Voltage measurements is already done in firmware but amps had to be measured using an multimeter in series with the power source. Later it would be nice to add this to the RPM board so it could be measured during flight.

## Thrust measurement
The firmware was adjusted to increase the trust from 0% to 93.75% in 16 steps. Each step period was set to four seconds so that thrust and amps could be manually written down. The cfclient was setup to log the PWM, voltage and RPM during the same time.

![thrust rpm thight](/docs/images/thrust_v_a_w_rpm_tight.png)

| Amps | Thrust (g) | Voltage | PWM (%) | Average RPM |
|---|---|---|---|---|
| 0.24 | 0.0 | 4.01 | 0 | 0 |
| 0.37 | 1.6 | 3.98 | 6.25 | 4485 |
| 0.56 | 4.8 | 3.95 | 12.5 | 7570 |
| 0.75 | 7.9 | 3.92 | 18.75 | 9374 |
| 0.94 | 10.9 | 3.88 | 25 | 10885 |
| 1.15 | 13.9 | 3.84 | 31.25 | 12277 |
| 1.37 | 17.3 | 3.80 | 37.5 | 13522 |
| 1.59 | 21.0 | 3.76 | 43.25 | 14691 |
| 1.83 | 24.4 | 3.71 | 50| 15924 |
| 2.11 | 28.6 | 3.67 | 56.25 | 17174 |
| 2.39 | 32.8 | 3.65 | 62.5 | 18179 |
| 2.71 | 37.3 | 3.62 | 68.75 | 19397 |
| 3.06 | 41.7 | 3.56 | 75 | 20539 |
| 3.46 | 46.0 | 3.48 | 81.25 | 21692 |
| 3.88 | 51.9 | 3.40 | 87.5 | 22598 |
| 4.44 | 57.9 | 3.30 | 93.75 | 23882 |

## Observations
As can be seen in the graph both then RPM and voltage vs. thrust is quadratic while the Power vs. thrust is linear. Also from the figures one can see that at 93.75% PWM the trust is about 58g. Using the values some interesting plots can be made. Like estimated flight time vs. battery capacity.

![flight time capacity](/docs/images/flighttime_capacity.png)


What can also be plotted and estimated is the PWM to thrust transfer function. The issue is that it is not the "PWM" that drives the motors but the voltage and amps = power. With the PWM it is possible to simulate different voltage levels so we are making the assumption that 50% PWM would result in 50% voltage. Then comes the issue that the battery has an internal resistant so that the voltage level wont be constant but will drop with the size of the load. This can be seen by plotting PWM and voltage. The graph is based on the values from the table above. The voltage supply used in this setup was a power box so the "internal resistance" of the battery is not really there but instead the resistance of the cables.

![pwm volt](/docs/images/pwm_volt.png)


So when we applying the PWM the voltage will be a percentage of the supply voltage which is based on the load which complicates things even more. Therefore two plots was made, one with voltage and one with PWM. The voltage that is applied to the motors is calculated as a percentage of the measured supply voltage. A strange thing here is that the fitting is a better match to the PWM then to the voltage. Something that has to be investigated further.

![pwm to thrust](/docs/images/pwm_to_thrust.png)


Another quite interesting curve is the rpm to thrust. The polyfit function gives the values: 1.0942e-07*rpm^2 - 2.1059e-04*rpm + 1.5417e-01.

![rpm thrust](/docs/images/rpm_thrust.png)
