---
title: DFU update of the STM32F405
page_id: dfu
redirects:
 - /docs/building-and-flashing/dfu/
---


This page explains how to flash bin files over the micro usb connection.

__The DFU (Device Firmware Update) mode should mainly be considered as a recovery mode in
which you can load new firmware to the STM32F405 MCU or as a mass firmware update mode. Do not use it if you do not know what it is__

The OTA (Over The
Air) update mode is much safer and more convenient, it can also update the
nRF51 MCU at the same time.


## Linux (Ubuntu)


### Automatic DFU Flashing
This method requires that the Crazyflie is turned on and running normally. The automatic DFU update mode will automatically put the Crazyflie into DFU mode, flash the new firmware to the correct address, and reboot the Crazyflie into the new firmware.

Install the dfu-util if you don\'t already have it with apt-get

    apt-get install dfu-util

Compile the Crazyflie firmware as normal with `make`.

Now power on the Crazyflie and connect it to your computer via a micro-USB cable.

Next, run the following from the root of the Crazyflie firmware to begin flashing. Do not unplug the Crazyflie, it will reboot when flashing has finished.

    make flash_dfu

Once the Crazyflie has rebooted and the command has finished executing, you can disconnect the Crazyflie from your computer and use it as normal.

### Manual DFU Flashing
If the Crazyflie cannot boot into the normal firmware, or some other issue prevents the automatic method from putting the Crazyflie into DFU mode, we will have to manually put the Crazyflie into DFU mode and flash the cf2.bin file.

First compile the firmware as normal with `make`.

Now it is time to boot the STM32F405 in the DFU update mode

     1. Connect a micro-USB cable to you computer but not yet to the Crazyflie 2.X
     2. Disconnect the Crazyflie 2.X battery if it is connected
     3. Now hold down the push button (on/off) while inserting the USB cable in the Crazyflie 2.X
     4. Hold down the button for about 5 seconds until you reach the second blink rate (1Hz). Then you can release the button.
     5. The STM32F405 is now in DFU mode


With the STM32F405 in DFU mode you should be able to find it with lsusb

    lsusb
    ...
    Bus XXX Device XXX: ID 0483:df11 STMicroelectronics STM Device in DFU Mode
    ...

Now we can flash the compiled cf2.bin file by running the following

    make flash_dfu_manual

This will flash the binary _after the bootloader_ at address 0x08004000.
Details for what command actually does the flashing can be found in the Makefile under the 
target `flash_dfu_manual`

Once the utility has finished downloading the firmware to the Crazyflie, you can disconnect the USB and battery and you're are good to go!

### Bootloader recovery

If for some reason, the dfu-utils overflashed the bootloader by flashing the firmware on the wrong address, you can recover the bootloader by getting the [latest release bootloader bin file](https://github.com/bitcraze/crazyflie2-stm-bootloader/releases). The bootloader can then be correctly flashed by manually putting the Crazyflie into DFU mode and running this command in the terminal.

    sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D cf2loader-1.0.bin

### DFU file

To flash with a .dfu file

    sudo dfu-util -d 0483:df11 -a 0 -D file.dfu

### Output

It takes a couple of seconds and the output should look something like
this

    cf@bitcraze:~/projects/crazyflie-firmware$make flash_dfu_manual
    tools/kbuild/Makefile.kbuild:147: warning: overriding recipe for target 'flash_dfu_manual'
    Makefile:172: warning: ignoring old recipe for target 'flash_dfu_manual'
    dfu-util -d 0483:df11 -a 0 -s 0x08004000:leave -D cf2.bin 
    dfu-util 0.9

    Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
    Copyright 2010-2016 Tormod Volden and Stefan Schmidt
    This program is Free Software and has ABSOLUTELY NO WARRANTY
    Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

    dfu-util: Invalid DFU suffix signature
    dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
    Opening DFU capable USB device...
    ID 0483:df11
    Run-time device DFU version 011a
    Claiming USB DFU Interface...
    Setting Alternate Setting #0 ...
    Determining device status: state = dfuERROR, status = 10
    dfuERROR, clearing status
    Determining device status: state = dfuIDLE, status = 0
    dfuIDLE, continuing
    DFU mode device DFU version 011a
    Device returned transfer size 2048
    DfuSe interface name: "Internal Flash  "
    Downloading to address = 0x08004000, size = 243176
    Download        [=========================] 100%       243176 bytes
    Download done.
    File downloaded successfully
    Transitioning to dfuMANIFEST state

Now you can disconnect the USB, connect the battery, and you are ready
to go. (Actually the only way to reset it is to disconnect the power, if
e.g. the battery wasn\'t disconnected while entering DFU mode)
