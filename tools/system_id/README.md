# System ID including PWM, RPM, Thrust, Voltage, Current, and Power

This folder contains scripts to measure propellers and motors using the systemId deck.

## Setup

* Load cells:
  * 100g: https://www.sparkfun.com/products/14727
  * 500g: https://www.sparkfun.com/products/14728
  * 1kg: https://www.adafruit.com/product/4540
* SystemID board with the following ICs
  * NAU7802 (thrust)
  * ACS37800 (power)
  * QRD1114 (RPM)
* Calibration weights
* M2/M3 Nylon screws
* CF Mount (3D printed)

Mount CF upside down (to avoid downwash)

## Measurement

### Common

1. Mount CF
2. Switch to crazyflie-lib-python with branch `feature-emergency-stop` in order to be able to CTRL+C a script safely.
3. Build firmware with configuration `make sysid_defconfig`
4. Flash firmware
5. Run `python3 calibscale.py --uri <URI>` and follow the prompts to calibrate the load cell. This will create an output file `calibration.yaml` with the calibration data. The other scripts will read this file (other name can be specified as command line argument).

### Ramping Motors

This test will ramp the motors and write collected data to `data.csv`.

```
python3 collect_data.py --uri <URI>
```

The results can be visualized with `plot_data.py`.

### Find (PWM,VBat)->Force, (desired Force,VBat)->PWM, (PWM, VBat)-> MaxForce

This test will randomly select sample a PWM value and average the resulting force and measured vbat over a short period of time. Then, it will output the largest possible PWM value to estimate the maximum force that the Crazyflie can create at the current state-of-charge. The mapping functions can be automatically computed.

```
python3 collect_data_max_thrust.py --uri <URI>
python3 system_id.py
```

### Motor Delay

This test uses a higher sampling rate and visualizes the delayed motor response given a step input.

```
python3 collect_data_motor_delay.py --uri <URI>
python3 plot_motor_delay.py
```

### Efficency

This test will staircase the motors and write collected data to `data.csv`.

```
python3 collect_data_efficiency.py --uri <URI>
```

The results can be visualized with `plot_data_efficiency.py`.
