---
title: Flashing
page_id: flashing
---

Writing a new binary to the Crazyflie is called flashing (writing it to the flash memory). This page describes how to flash from the command line and there are a few different ways to do it.

## Using Crazyradio

The most common way to flash is probably to use the Crazyradio.

### prerequisites
* A Crazyradio with drivers installed
* [crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python) placed on the same directory level in the file tree
* The firmware has been built
* The current working directory is the root of the frazyflie-firmware project

### Manually entering bootloader mode

* Turn the Crazyflie off
* Start the Crazyflie in bootloader mode by pressing the power button for 3 seconds. Both the blue LEDs will blink.
* In your terminal, run `make cload`

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

### Automatically enter bootloader mode

* Add the address of the crazyflie to the [`config.mk`](/building-and-flashing/configure_build/) file, for instance `CLOAD_CMDS = -w radio://0/80/2M`
* Make sure the Crazyflie is on
* In your terminal, run `make cload`

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.

## Using a debug adapter

You need:

* An ST Link V2 Debugger
* open ocd installed ([installation intructions](/development/openocd_gdb_debugging/))
* The firmware has been built
* The current working directory is the root of the frazyflie-firmware project

In your terminal, run

`make flash`
