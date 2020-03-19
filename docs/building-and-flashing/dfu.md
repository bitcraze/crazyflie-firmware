---
title: DFU update of the STM32F405
page_id: dfu
---

The DFU update mode should mainly be considered as a recovery mode in
which you can load new firmware to the STM32F405 MCU. The OTA (Over The
Air) update mode is much more convenient and it can also update the
nRF51 MCU at the same time.

### Linux (Ubuntu)

Install the dfu-util if you don\'t already have it with apt-get

    apt-get install dfu-util

Now it is time to boot the STM32F405 in the DFU update mode

     1. Connect a micro-USB cable to you computer but not yet to the Crazyflie 2.0
     2. Disconnect the Crazyflie 2.0 battery if it is connected
     3. Now hold down the push button (on/off) while inserting the USB cable in the Crazyflie 2.0
     4. Hold down the button for about 5 seconds until you reach the second blink rate (1Hz). Then you can release the button.
     5. The STM32F405 is now in DFU mode
     

With the STM32F405 in DFU mode you should be able to find it with lsusb

    lsusb
    ...
    Bus XXX Device XXX: ID 0483:df11 STMicroelectronics STM Device in DFU Mode
    ...

**BIN file**

Now the STM32F405 can be updated. Currently we only build binary files
.bin and not .dfu files so we need to specify more things to dfu-util.
If the Crazyflie 2.0 firmware was compiled with CLOAD=1 (default option)
the binary should be flashed after the bootloader at address 0x08004000.

    sudo dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D cflie.bin

If the Crazyflie 2.0 firmware was compiled with CLOAD=0 the binary
should be flashed in the beginning of the flash (address 0x08000000).
WARNING, this will overwrite the radio-bootloader if it is there. You
can however flash the bootloader back this same way with DFU.

    sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D cflie.bin

**DFU file**

To flash with a .dfu file

    sudo dfu-util -d 0483:df11 -a 0 -D file.dfu

**Output**

It takes a couple of seconds and the output should look something like
this

    cf@bitcraze:~/projects/crazyflie-firmware$ sudo dfu-util -a 0 -s 0x08004000 -D cflie.bin
    dfu-util 0.5

    (C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.
    (C) 2010-2011 Tormod Volden (DfuSe support)
    This program is Free Software and has ABSOLUTELY NO WARRANTY

    dfu-util does currently only support DFU version 1.0

    Opening DFU USB device... ID 0483:df11
    Run-time device DFU version 011a
    Found DFU: [0483:df11] devnum=0, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg"
    Claiming USB DFU Interface...
    Setting Alternate Setting #0 ...
    Determining device status: state = dfuERROR, status = 10
    dfuERROR, clearing status
    Determining device status: state = dfuIDLE, status = 0
    dfuIDLE, continuing
    DFU mode device DFU version 011a
    Device returned transfer size 2048
    No valid DFU suffix signature
    Warning: File has no DFU suffix
    DfuSe interface name: "Internal Flash  "

Now you can disconnect the USB, connect the battery, and you are ready
to go. (Actually the only way to reset it is to disconnect the power, if
e.g. the battery wasn\'t disconnected while entering DFU mode)
