---
title: Building and Flashing
page_id: build
---

## Dependencies

You'll need to use either the [Crazyflie VM](https://github.com/bitcraze/bitcraze-vm),
[the toolbelt](https://github.com/bitcraze/toolbelt) or
install some ARM toolchain.

There is also experimental support for [building with nix](./nix.md).
This allows to build the firmware in a reproducable way.

### Install a toolchain

#### Toolchain and compiler version policy
Our toolchain policy is to support the two most recent [Ubuntu Long Term Support (LTS) releases](https://wiki.ubuntu.com/Releases). This ensures we stay aligned with widely used, actively maintained versions. As of October 15th, 2024, the oldest supported LTS release is Ubuntu 22.04 (Jammy Jellyfish), which includes `gcc-arm-none-eabi` version 10.3.

This means that if the firmware can not be compiled using gcc 10.3, **or anything newer**, it should be considered a bug.

##### Ubuntu
```
$ sudo apt-get install make gcc-arm-none-eabi
```

##### macOS
The required toolchain on macOS can be installed using [homebrew](https://brew.sh/)

```
$ brew install gcc-arm-embedded
```

##### Windows
The supported way to build the Crazyflie on Windows is to use the Windows Subsystem for Linux (WSL) on Windows 10+.
This means that developement happens in a Linux environment.
Flashing is handled by installing Python and the Crazyflie client on Windows launched from linux.

To get started you need to [enable WSL and install an Ubuntu system](https://docs.microsoft.com/en-us/windows/wsl/install).
This can be done by opening `power shell` as administrator and typing:

```
$ wsl --install
```

Then follow the [install instruction for Ubuntu 20.04](#ubuntu) above to install the required build dependencies.

For [flashing](#flashing) you need to install [Python](https://www.python.org/downloads/windows/) (=>version 3.7) and the [CFclient](https://github.com/bitcraze/crazyflie-clients-python) **on Windows**.
When installing Python, the checkbox to add python to the Path should be checked and then the CFclient can be installed with pip in a `powershell` or `cmd` window:
```
$ pip.exe install cfclient
```

The Crazyflie makefile will automatically use the Windows python when running in WSL.

### Cloning

This repository uses git submodules. Clone with the `--recursive` flag

```
$ git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
```

> Note: Make sure there are no spaces in the folder structure leading up to the repository (example: _/a/path/with/no/spaces/crazyflie-firmware/_ vs _a/path with spaces/crazyflie-firmware/_). Our build system can not handle file system paths with spaces in them, and compilation will fail.

If you already have cloned the repo without the `--recursive` option, you need to
get the submodules manually

```
$ cd crazyflie-firmware
$ git submodule init
$ git submodule update
```


## Compiling

### Configuration

Before you can build the firmware, you will need to configure it. To use a platform default configuration, run the following command:

#### Crazyflie 2.0, Crazyflie 2.1(+)
```
$ make cf2_defconfig
```
#### Crazyflie 2.1 Brushless
```
$ make cf21bl_defconfig
```
#### Crazyflie Bolt
```
$ make bolt_defconfig
```
#### Wearable
Wearable is a configuration intended for a Crazyflie with a Lighthouse deck that is used as a general purpose positioning device.
```
$ make wearable_defconfig
```

### Building the firmware
Then build the firmware with:

#### Linux/WSL
```
$ make -j$(nproc)
```
#### macOS
```
$ make -j$(sysctl -n hw.ncpu)
```

>  Alternatively, to configure and build with the toolbelt, prepend `tb` to the make commands as follows:
>
>  ```
>  $ tb make cf2_defconfig
>  $ tb make
>  ```

Build artifacts, including binaries, will end up in the `build` directory.

### Customize the firmware with kbuild (Advanced)


**Please note** that these instructions are only meant for you if you want to build an custom firmware for a custom platform than the ones we have listed above. You can still configure and change the firmware without kbuild.

Please go to [these instructions](/docs/development/kbuild.md) to learn how to use the menuconfig.


### Build python bindings

There are certain functions, like the high level commander and controllers, that have been wrapped to python bindings. These can be used to easily test these functions on a computer or use it in a simulation.

First make sure that you have [SWIG](https://swig.org/) installed on your system. Then execute the following commands in the terminal

```
$ make cf2_defconfig
$ make bindings_python
$ cd build
$ python3 setup.py install --user
```

## Make targets

### General targets
```
all             : Shortcut for build
compile         : Compile cflie.hex. WARNING: Do NOT update version.c
build           : Update version.c and compile cflie.elf/hex
clean_o         : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean           : Clean every compiled files
mrproper        : Clean every compiled files and the classical editors backup files

cload           : If the crazyflie-clients-python is placed on the same directory level and
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware
             using the wireless bootloader.
flash           : Flash .elf using OpenOCD
halt            : Halt the target using OpenOCD
reset           : Reset the target using OpenOCD
openocd         : Launch OpenOCD
rtt             : Start RTT server. Compile the firmware with "DEBUG_PRINT_ON_SEGGER_RTT=1"
             and the console is visible over TCP on port 2000 "telnet localhost 2000".
bindings_python : Build the python bindings for firmware wrappers
```

### Noteable Kbuild targets
```
menuconfig         : Open up a terminal user interface to set configuration options
defconfig          : Generate a `.config` with the default configuration options
cf2_defconfig      : Merge configuration options from `configs/cf2_defconfig` with default
cf21bl_defconfig   : Merge configuration options from `configs/cf21bl_defconfig` with default
tag_defconfig      : Merge configuration options from `configs/tag_defconfig` with default
bolt_defconfig     : Merge configuration options from `configs/bolt_defconfig` with default
wearable_defconfig : Merge configuration options from `configs/wearaböe_defconfig` with default
allyesconfig       : Generate a `.config` with the all configuration options enabled
allnoconfig        : Generate a `.config` with the all configuration options disabled
randconfig         : Generate a `.config` with random valid values to all configuration options
```

## Flashing
Writing a new binary to the Crazyflie is called flashing (writing it to the flash memory). This page describes how to flash from the command line and there are a few different ways to do it.

### Using Crazyradio

The supported way to flash when developping for the Crazyflie is to use the Crazyradio and the radio bootloader.

#### Prerequisites
* A Crazyradio with drivers installed
* [Crazyflie Client installed](https://github.com/bitcraze/crazyflie-clients-python) with Python's pip (so not by Snap (Ubuntu) or the .exe (Windows))
  * Note than when developping in WSL on Windows, the client needs to be installed on Windows. See the [Windows build instruction](#windows) above.
* The firmware has been built
* The current working directory is the root of the crazyflie-firmware project

#### Manually entering bootloader mode

* Turn the Crazyflie off
* Start the Crazyflie in bootloader mode by pressing the power button for 3 seconds. Both the blue LEDs will blink.
* In your terminal, run

```
$ make cload
```

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

#### Automatically enter bootloader mode
* Make sure the Crazyflie is on
* In your terminal, run `CLOAD_CMDS="-w [CRAZYFLIE_URI]" make cload`
* or run `cfloader flash build/cf2.bin stm32-fw -w [CRAZYFLIE_URI]`
with [CRAZYFLIE_URI] being the uri of the crazyflie.

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.

### Using a debug adapter

You need:

* An ST Link V2 or V3 Debugger
* open ocd installed ([installation intructions](/docs/development/openocd_gdb_debugging.md))
* The firmware has been built
* The current working directory is the root of the crazyflie-firmware project

In your terminal, run

```
$ make flash
```

## Unit testing

### Running all unit tests

With the environment set up locally

```
$ make unit
```

with the docker builder image and the toolbelt

```
$ tb make unit
```

### Running one unit test

When working with one specific file it is often convenient to run only one unit test

```
$ make unit FILES=test/utils/src/test_num.c
```

or with the toolbelt

```
$ tb make unit FILES=test/utils/src/test_num.c
```

### Running unit tests with specific build settings

Defines are managed by make and are passed on to the unit test code. Use the
normal ways of configuring make when running tests. For instance to run test
for Crazyflie 1

```
$ make unit LPS_TDOA_ENABLE=1
```

### Dependencies

Frameworks for unit testing and mocking are pulled in as git submodules.

The testing framework uses ruby and rake to generate and run code.

To minimize the need for installations and configuration, use the docker builder
image (bitcraze/builder) that contains all tools needed. All scripts in the
tools/build directory are intended to be run in the image. The
[toolbelt](https://github.com/bitcraze/toolbelt) makes it
easy to run the tool scripts.
