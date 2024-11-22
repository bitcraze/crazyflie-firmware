---
title: ESC Configuration
page_id: esc_configuration
---

## Configure the ESCs of the Crazyflie 2.1 Brushless

***NOTE: You must use a Chrome browser (Chromium or similar does not work)***

Open the wep page [esc-configurator](https://esc-configurator.com) in a Chrome browser.

Connect the Crazyflie 2.1 Brushless using the provided USB cable.
Select a serial port in the GUI.
If you are working on a Linux OS you might need to add USB permissions like this:
[USB permissions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)

The serial port should be named something along the lines of "CrazyflieXX"

Connect to the pass through interface via USB by pressing the Connect button.
![connect](/docs/images/esc_configurator_connect.png)

### Motor control
The first page is a motor control page where you can check that the motors are spinning as expected.
Enable control by checking the "Enable motor control" checkbox.

This control overrides the software arming.

You can either spin the motors separately or all motors at the same time with the "Master speed" slider.
![motor control](/docs/images/esc_motor_control.png)

***NOTE: Be careful to not ramp the speed up so high that the drone takes off or flips.***


### Write settings to the ESCs
Click the "Read Settings" button. The settings read from the ESCs will now be displayed.

Be aware that setting other settings than the recommended is done at you own risk, make sure you know what you are doing before attempting to change these.

The default setting configuration we have written to the ESCs at production are;
![default config](/docs/images/esc_default_config.png) for our default settings.

To write the setting to the ESCs press the "Write settings" button. This will write **all** the settings shown in the GUI to the ESCs.

***NOTE if you push "reset to default" these settigns are not our default settings, but the ones specified by the esc-configurator website.***


### Flash the ESCs
To flash the ESCs press the "Flash Firmware to this ESC".

You can choose to flash either a standard off-the-shelf FW like Bluejay or BLHEli_S or a custom FW.

We have our own fork of Bluejay which you can find here;
https://github.com/bitcraze/bluejay


***NOTE: If you use a FW that has a PWM Frequency setting please note that running on 24kHz causes unstable flight behaviour***

