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
3. Build firmware with configuration `make sysid_defconfig` and `make -j$(nproc)`
4. Flash firmware by bringing the Crazyflie into flash mode and running `make cload`
5. Run `python3 calibscale.py --uri <URI>` and follow the prompts to calibrate the load cell. This will create an output file `calibration.yaml` with the calibration data. The other scripts will read this file (other name can be specified as command line argument).

### Collecting data

There are multiple modes for collecting data. Each of the modes can easily be accessed by giving the correct argument to 'collect_data.py'. The combination of propellers, motors, and battery needs to be given by COMB. Since COMB can have a dash, you can use the following command in your terminal:

```
python3 collect_data.py --uri=<URI> --mode=<MODE> --comb=<COMB> 
```

The collected data is then saved accordingly to `data_<MODE>_<COMB>.csv`.

In theory, you can put whatever you want for COMB or even leave it empty. We've come up with the following scheme to distinguish different setups easily:

| Chars                   | Option 1    | Option 2    | Option 3      |
| --------                | -------     | -------     | -------       |
| 1: Propeller type       | old: -      | 2.1+: +     |
| 2: Motor type           | 17mm: B     | 20mm: T     | Brushless: L  |
| 3-5: Battery capacity   | 250mAh: 250 | 350mAh: 350 |

As an example: The Crazyflies 2.1+ gets shipped with the new propellers, 17mm motors, and a 250mAh battery, so we would set `COMB=+B250`.

### Ramping Motors

This test will simply ramp the motors up and down. The results can be visualized with the correcsponding plot script.

```
python3 collect_data.py --uri <URI> --mode=ramp_motors --comb=<COMB> 
python3 plot_data_ramp_motors.py
```

### Find (PWM,VBat)->Force, (desired Force,VBat)->PWM, (PWM, VBat)-> MaxForce

This test will randomly select sample a PWM value and average the resulting force and measured vbat over a short period of time. Then, it will output the largest possible PWM value to estimate the maximum force that the Crazyflie can create at the current state-of-charge. The mapping functions can be automatically computed. TODO 

```
python3 collect_data.py --uri <URI> --mode=max_thrust --comb=<COMB> 
python3 system_id.py
```

### Motor Delay

This test uses a higher sampling rate and visualizes the delayed motor response given a step input.

```
python3 collect_data.py --uri <URI> --mode=motor_delay --comb=<COMB> 
python3 plot_data_motor_delay.py
```

### Efficency

This test will staircase the motors.

```
python3 collect_data.py --uri <URI> --mode=efficiency --comb=<COMB> 
python3 plot_data_efficiency.py
```