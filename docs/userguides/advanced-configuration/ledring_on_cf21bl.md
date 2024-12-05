---
title: LED-ring on Crazyflie 2.1 Brushless
page_id: ledring_on_cf21bl
---

## How to get the LED-ring working on Crazyflie 2.1 Brushless

Due to a resource conflict between the DSHOT motor signal driver and the LED-ring (they both use DMA1 Stream 6), DSHOT can not be used together with the LED-ring.
The default ESC firmware we use is [Bluejay](https://github.com/bird-sanctuary/bluejay) (a successor of BLHeli_S) which has removed support for all other motor signal protocols than DSHOT. A work-around then is to use BLHeli_S and e.g. OneShot125 protocol. However there are more complications as a 24KHz ESC PWM seem to interfere with the gyro, so a special version of BLHeli_S that supports 48KHz should be used. This version is called BLHeli_M.

### Download the BLHeli_M O_H_10_48_REV16_77.hex

The ESC firmware file O_H_10_48_REV16_77.hex can be found [here](https://github.com/JazzMaverick/BLHeli/blob/JazzMaverick-patch-1/BLHeli_S%20SiLabs/Hex%20files%2016.77%2048k/O_H_10_48_REV16_77.HEX). Download it to a local folder. 

### Flash the ESCs

Use the [ESC-configuration guide](/docs/userguides/advanced-configuration/esc_configuration.md) to flash custom FW to the ESCs

![Flash custom FW](/docs/images/esc-flash-local-firmware.png)

### Write settings to the ESCs

We recommend the settings in the picture below:

![BLHeli_M recommended settings](/docs/images/esc-blheli_m_settings.png)

To write the setting to the ESCs press the "Write settings" button. This will write **all** the settings shown in the GUI to the ESCs.

***NOTE if you push "reset to default" these settigns are not our default settings, but the ones specified by the esc-configurator website.***

## Configure the Crazyflie 2.1 brushless firmware to output OneShot125/Oneshot42 motor signal

Use the build and flash firmware guide to learn how to build and flash firmware

Build the firmware with OneShot125/OneShot42 motor signal protocol:
```
$ make cf21bl_defconfig
$ make menuconfig
```

![Flash custom FW](/docs/images/esc_menuconfig_oneshot125.png)

Then build and flash the Crazyflie 2.1 Brushless firmware. The LED-ring should now work with your Crazyflie 2.1 Brushless.

## Restoring the original ESC firmware

We use different settings then the official Bluejay firmware, so use our fork of the Bluejay ESC firmware. Our latest release can be found [here](https://github.com/bitcraze/bluejay/releases). Download and flash the cfbl2.1_esc_normal_m1-m3.hex on all ESCs. You can configure the rotation direction afterwards together with the other settings that are shown in the [ESC-configuration guide](/docs/images/esc_configuration-md).
