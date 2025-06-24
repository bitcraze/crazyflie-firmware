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
* Loadcell mount (3D printed, optional)

Mount CF upside down (to avoid downwash)

## Measurement

### Common

1. Mount CF
1. Build firmware with configuration `make sysid_defconfig` and `make -j$(nproc)`
1. Flash firmware by bringing the Crazyflie into flash mode and running `make cload`
1. Run `python3 calibscale.py --uri <URI>` and follow the prompts to calibrate the load cell. This will create an output file `calibration.yaml` with the calibration data. The other scripts will read this file (other name can be specified as command line argument). After changing the battery, you don't have to do a whole new calibration. Instead, you can simply set the 0 value for the new battery. It is assumed that the slope of the calibration is the same.

### Ramping Motors

This test will simply ramp the motors up and down. The results can be visualized with the correcsponding plot script. Is is not really needed and just for fun. 

```
python3 collect_data.py --uri <URI> --mode ramp_motors --comb <COMB> 
python3 plot_data_ramp_motors.py --file <FILENAME>
```

### System identification

The system identification is done in three steps, each called a different mode: First, the static thrust parameters are identified. Those can then be updated in the firmware and verified in the second step. Lastly, the dynamic parameters can be identified. 

The scheme for all data collection and identification script is the same:
```
python3 collect_data.py --uri <URI> --mode <MODE> --comb <COMB> 
python3 system_id.py --mode <MODE> --comb <COMB> 
```

The collected data is then saved accordingly to `data_<MODE>_<COMB>.csv`.

The combination of propellers, motors, and battery needs to be given by COMB. In theory, you can put whatever you want for COMB or even leave it empty. We've come up with the following scheme to distinguish different setups easily:

The motors and propellers are given by the first character. **L** for the legacy propellers with the 17mm motors. **P** for the 2.1+ propellers with the 17mm motors. **T** for the thrust upgrade kit propellers and 20mm motors. **B** for the brushless setup. The number following is simply the capacity in mAh of the used battery. As an example: The Crazyflies 2.1+ gets shipped with the new propellers, 17mm motors, and a 250mAh battery, so we would set `COMB=P250`.

#### Static parameters
`mode = static`

In this mode, the motors are given random commands in a valid range and all the important data is stored. From that we can calculate the most important curve: Vmotors [V] -> Thrust [N], where Vmotors = PWM_CMD / PWM_MAX * Vbat. This curve helps us to calculate the needed PWM_CMD from a thrust command and the current battery voltage Vbat. Additionally, the RPM -> Thrust [N] curve and the efficiency is identified.

The important parameters will be stored in `params_<COMB>.yaml`. We advise to take multiple datasets from multiple different drones for best results.

#### Verification

To verify the parameters, we need to add the new set of values to the firmware. In `motors.c`, we can find the arrays for the battery compensation called the same as in the yaml file (p_vmotor2thrust). After adding the new values and flashing, we run the data collection `mode = static_verification`, where the motors are again given random commands, but the thrust is battery compensated. Note: You need to set the correct compensation mode in the `sysid_defconfig`!

In the system_id part, we should see all values beeing on the plane in the first plot or the line in the second plot. That means the battery compensation works.

#### Dynamic parameters

Lastly, we want to know how fast the motors can change speed/thrust. In `mode = dynamic`, data is first collected with motors changing speed from lowest PWM to highest PWM command, once with and once without battery compensation.

In the system_id part, we can observe the thrust dynamics. They can be modelled in two ways. From first principles we know the torque of a DC motor beeing $T=k\frac{V}{R}-k^2 \omega$ (in steady state, neglecting electrical dynamics) and the differential equation for the motor and propeller speed beeing $T=J\dot{\omega} + k_T \omega^2$. Rearranging the terms and ignoring the drag part ($k_T \omega^2$), we get a first order system for the propeller dynamics as: $\dot{RPM} = \frac{1}{\tau_{RPM}} (RPM_{CMD}-RPM)$. Alternatively, we can set up the thrust directly as first order system, which is also done in some works: $\dot{f} = \frac{1}{\tau_f} (f_{CMD}-f)$. We identify both parameters and save them in the same file as before.