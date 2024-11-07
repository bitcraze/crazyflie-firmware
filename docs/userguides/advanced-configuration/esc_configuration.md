# Using ESC-configurator web tool

***NOTE: You must use a Chrome browser (Chromium or similar does not work)***

Open the wep page esc-configurator.com

Connect the Crazyflie 2.1 Brushless using the provided USB cable. 
Select a serial port, not that you might need to add some udev rules if you are working on a linux.  ADD UDEV RULES

The serial port should be named something in the lines of "Crazyflie..."

Connect to the pass through interface via USB by pressing the Connect button.
![connect](/docs/images/esc_configurator_connect.png)

## Motor control 
The first page is a motor control page where you can check that the motors seem to be spinning as expected.
Enable control by checking the "Enable motor control" checbox.

You can either spin motor by motor or all motors at the same time with the "Master speed" slider.
![motor control](/docs/images/esc_motor_control.png)
***NOTE: Be careful to not ramp the speed up so high it takes of or flips.*** 

 This control overrides the software arming and can therefore spin out of control.

## Write settings to the ESCs
Click the "Read Settings" button.

The settings we recommend are the following;
![default config](/docs/images/esc_default_config.png)

NOTE: Setting other settings than the recommended is at you own risk, make sure you know what you are doing before attempting to change these

*NOTE if you reset to default these settigns are not our default settings
The settings shown in the gui on the left side are not our default settings read from the ESC FW. 
Writing these will cause unexpected behaviour.*

To write the setting to the ESCs press the "Write settings" button.
This will write all the settings in the GUI to the ESCs.

## Flash the ESCs
To flash the ESCs presh the "Flash Firmware to this ESC".

You can choose to flash either a standard off-the-shelf FW like Bluejay or BLHEli_S or a custom FW.

We have our own fork of Bluejay which you can find here;
https://github.com/bitcraze/bluejay


***NOTE: If you use an off-the-shelf FW that has a PWM Frequency setting please note that running on 24kHz causes unstable flight behaviour***

