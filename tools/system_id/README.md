# System ID including PWM, RPM, Thrust, Voltage, Current, and Power

This folder contains scripts to measure propellers and motors using the systemId deck. Some more information can be found in those two blog articles:

- [Building a Crazyflie thrust stand (2021)](https://www.bitcraze.io/2021/08/building-a-crazyflie-thrust-stand/)
- [Keeping Thrust Consistent as the Battery Drains (2025)](https://www.bitcraze.io/2025/10/keeping-thrust-consistent-as-the-battery-drains/)

## Setup

* Load cells:
  * 100g: https://www.sparkfun.com/products/14727
  * 500g: https://www.sparkfun.com/products/14728
  * 1kg: https://www.adafruit.com/product/4540
  * 6 axis (for torque measurements only): https://www.ati-ia.com/products/ft/ft_models.aspx?id=nano43
* SystemID board with the following ICs
  * NAU7802 (thrust)
  * ACS37800 (power)
  * QRD1114 (RPM)
* Calibration weights
* M2 & M3 screws
* CF Mount (3D printed)
* Loadcell mount (3D printed)

Mount CF upside down (to avoid downwash effects)

## Measurement

### Common

1. Mount CF
1. Merge `sysid.conf` with your platform default config: `./scripts/kconfig/merge_config.sh configs/sysid.conf configs/cf2_defconfig` (or `cf21bl_defconfig`, etc.)
1. Compile (`make -j$(nproc)`)
1. Flash firmware by bringing the Crazyflie into flash mode and running `make cload`
1. Go into `cd tools/system_id`
1. Run `python3 calibscale.py --uri <URI>` and follow the prompts to calibrate the load cell. This will create an output file `calibration.toml` (other name can be specified as command line argument) with the calibration data. The other scripts will read this file. After changing the battery, you don't have to do a whole new calibration. Instead, you can simply set the 0 value for the new battery. It is assumed that the slope of the calibration is the same.

In the following, to distinguish between different crazyflie setups, we introduce the combination of propellers, motors, and battery as command line argument `COMB`. In theory, you can put whatever you want for COMB or even leave it empty. We've come up with the following scheme to distinguish different setups easily:

The motors and propellers are given by the first character. **L** for the legacy propellers with the 16mm motors. **P** for the 2.1+ propellers with the 16mm motors. **T** for the thrust upgrade kit propellers and 20mm motors. **B** for the brushless setup. The number following is simply the capacity in mAh of the used battery. As an example: The Crazyflies 2.1+ gets shipped with the new propellers, 16mm motors, and a 250mAh battery, so we would set `--COMB P250`.

If you have an ATI load cell for **torque** measurements, you first need to set the correct interface name in the script. To find that, you can run `loadcell.py`. Note that you have to run the script as root or allow access, see comments in `loadcell.py`. Then you can set the command line argument `--use_loadcell` to use the data from the loadcell and not from the deck. Note that no calibration is needed. Since measuring torques takes longer, we collect no torque data by setting that flag. For that, set the additional flag `--torques`. If you don't have the load cell, don't worry! The torque data will simply be set to zeros.

### Ramping Motors

This test will simply ramp the motors up and down. The results can be visualized with the corresponding plot script. It's not really needed and just for fun. 

```
python3 collect_data.py --uri <URI> --mode ramp_motors --comb <COMB> 
python3 plot_data_ramp_motors.py --file data_ramp_motors_<COMB>.csv
```

### System identification

The system identification is done in three steps, each called a different mode: First, the static thrust parameters are identified. Those can then be updated in the firmware and verified in the second step. Lastly, the dynamic parameters can be identified. 

The scheme for all data collection and identification script is the same:
```
python3 collect_data.py --uri <URI> --mode <MODE> --comb <COMB> 
python3 system_id.py --mode <MODE> --comb <COMB> 
```

The collected data is then saved accordingly to `data_<MODE>_<COMB>.csv`.

#### Static parameters
`--mode static`

In this mode, the motors are given random commands in a valid range and all the important data is stored. From that, we can calculate the most important curve: Vmotors [V] -> Thrust [N], where Vmotors = PWM_CMD / PWM_MAX * Vbat. This curve helps us to calculate the needed PWM_CMD from a thrust command and the current battery voltage Vbat. Additionally, the RPM -> Thrust [N] curve and the efficiency is identified. 

The important parameters will be stored in `params_<COMB>.toml`. We advise to take multiple datasets from multiple different drones for best results. The new parameters need to be added to the firmware. In the `platform_defaults_<PLATFORM>.h`, we can find the constants for the battery compensation called the same as in the toml file (vmotor2thrust). Then, we obviously need to build the firmware and flash the drone again.

#### Verification

To verify the parameters, after adding the new values and flashing, we run the data collection `mode = static_verification`, where the motors are again given random commands, but the thrust is battery compensated. In the system_id part, we should see all values beeing on the plane in the first plot or the line in the second plot. That means the battery compensation works and the behavior is linear.

#### Dynamic parameters

Lastly, we want to know how fast the motors can change speed/thrust. In `mode = dynamic`, data is first collected with motors following a reference for system identification with battery compensation enabled. Make sure to use a full battery for the data collection, since the trajectory is quite long.

In the system_id part, we can observe the thrust dynamics. They can be modelled in two ways. From first principles we know the torque of a DC motor beeing $T=k\frac{V}{R}-k^2 \Omega$ (in steady state, neglecting electrical dynamics) and the differential equation for the motor and propeller speed beeing $T=J\dot{\Omega} + k_T \Omega^2$. Rearranging the terms and abusing $\Omega$ as RPM, we get the a system of the form: $\dot{\Omega} = a\Omega_\mathrm{set} + b\Omega + c\Omega^2$, where $\Omega_\mathrm{set}$ is the setpoint known from the thrust curves. This equation has two problems: First, the thrust curves should be collected on multiple runs using different motors to get a good average thrust curve. This is not possible here and it will overfit the particular motor from the experiment. Second, the parameters ($a,b,c$) are most likely different for acceleration and deceleration, since some motor types cannot brake (due to the used electronics). Thus, we adjust the model to fix both issues: 

$$
\dot{\Omega} = \begin{cases}
    a\cdot(\Omega_\mathrm{set}-\Omega) + b\cdot(\Omega_\mathrm{set}^2-\Omega^2),  & \forall u_\Omega>\Omega. \\
    c\cdot(\Omega_\mathrm{set}-\Omega) + d\cdot(\Omega_\mathrm{set}^2-\Omega^2),  & \forall u_\Omega\leq\Omega.
\end{cases}
$$
Note: A good result here is dependent on a good fit earlier. 

For brushless motors, the even simpler form of $\dot{\Omega} = a(\Omega_\mathrm{set}-\Omega)$ also provides a relatively good fit, so this parameter is also computed. 

Alternatively, we can set up the thrust directly as first order system, which is also done in some works: $\dot{f} = \frac{1}{\tau_f} (f_{CMD}-f)$. We identify all parameters and save them in the same file as before. Also, the results of the fits are shown in plots and the console.
