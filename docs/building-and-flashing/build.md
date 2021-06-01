---
title: Building and Flashing
page_id: build
---

## Dependencies

You'll need to use either the [Crazyflie VM](https://wiki.bitcraze.io/projects:virtualmachine:index),
[the toolbelt](https://wiki.bitcraze.io/projects:dockerbuilderimage:index) or
install some ARM toolchain.

### Install a toolchain

#### OS X
```bash
brew tap PX4/homebrew-px4
brew install gcc-arm-none-eabi
```

#### Debian/Ubuntu

Tested on Ubuntu 14.04 64b/16.04 64b/18.04 64b/20.04 64b/20.10 64b:

For Ubuntu 14.04 :

```bash
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
```

For Ubuntu 16.04 and 18.04:

```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt install gcc-arm-embedded
```

For Ubuntu 20.04 and 20.10:

```bash
sudo apt-get install make gcc-arm-none-eabi
```

#### Arch Linux

```bash
sudo pacman -S community/arm-none-eabi-gcc community/arm-none-eabi-gdb community/arm-none-eabi-newlib
```

#### Windows

The GCC ARM Embedded toolchain for Windows is available at [launchpad.net](https://launchpad.net/gcc-arm-embedded/+download). Download the zip archive rather than the executable installer. There are a few different systems for running UNIX-style shells and build systems on Windows; the instructions below are for [Cygwin](https://www.cygwin.com/).

Install Cygwin with [setup-x86_64.exe](https://www.cygwin.com/setup-x86_64.exe). Use the standard `C:\cygwin64` installation directory and install at least the `make` and `git` packages.

Download the latest `gcc-arm-none-eabi-*-win32.zip` archive from [launchpad.net](https://launchpad.net/gcc-arm-embedded/+download). Create the directory `C:\cygwin64\opt\gcc-arm-none-eabi` and extract the contents of the zip file to it.

Launch a Cygwin terminal and run the following to append to your `~/.bashrc` file:
```bash
echo '[[ $PATH == */opt/gcc-arm-none-eabi/bin* ]] || export PATH=/opt/gcc-arm-none-eabi/bin:$PATH' >>~/.bashrc
source ~/.bashrc
```

Verify the toolchain installation with `arm-none-eabi-gcc --version`

### Cloning

This repository uses git submodules. Clone with the `--recursive` flag

```bash
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
```

If you already have cloned the repo without the `--recursive` option, you need to
get the submodules manually

```bash
cd crazyflie-firmware
git submodule init
git submodule update
```


## Compiling

### Crazyflie 2.X

This is the default build so just running ```make``` is enough or:
```bash
make PLATFORM=cf2
```

or with the toolbelt

```bash
tb make PLATFORM=cf2
```

### Roadrunner

Use the ```tag``` platform

```bash
make PLATFORM=tag
```

or with the toolbelt

```bash
tb make PLATFORM=tag
```


### config.mk
To create custom build options create a file called `config.mk` in the `tools/make/`
folder and fill it with options. E.g.
```
PLATFORM=CF2
DEBUG=1
```
More information can be found on the
[Bitcraze documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)


# Make targets:

```
all        : Shortcut for build
compile    : Compile cflie.hex. WARNING: Do NOT update version.c
build      : Update version.c and compile cflie.elf/hex
clean_o    : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean      : Clean every compiled files
mrproper   : Clean every compiled files and the classical editors backup files

cload      : If the crazyflie-clients-python is placed on the same directory level and
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware
             using the wireless bootloader.
flash      : Flash .elf using OpenOCD
halt       : Halt the target using OpenOCD
reset      : Reset the target using OpenOCD
openocd    : Launch OpenOCD
```

# Flashing
Writing a new binary to the Crazyflie is called flashing (writing it to the flash memory). This page describes how to flash from the command line and there are a few different ways to do it.

## Using Crazyradio

The most common way to flash is probably to use the Crazyradio.

### Prerequisites
* A Crazyradio with drivers installed
* [crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python) placed on the same directory level in the file tree
* The firmware has been built
* The current working directory is the root of the crazyflie-firmware project

### Manually entering bootloader mode

* Turn the Crazyflie off
* Start the Crazyflie in bootloader mode by pressing the power button for 3 seconds. Both the blue LEDs will blink.
* In your terminal, run `make cload`

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

### Automatically enter bootloader mode

* Add the address of the crazyflie to the [`config.mk`](#configmk) file, for instance `CLOAD_CMDS = -w radio://0/80/2M`
* Make sure the Crazyflie is on
* In your terminal, run `make cload`

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.

## Using a debug adapter

You need:

* An ST Link V2 Debugger
* open ocd installed ([installation intructions](/docs/development/openocd_gdb_debugging.md))
* The firmware has been built
* The current working directory is the root of the crazyflie-firmware project

In your terminal, run

`make flash`


# Unit testing

## Running all unit tests

With the environment set up locally

        make unit

with the docker builder image and the toolbelt

        tb make unit

## Running one unit test

When working with one specific file it is often convenient to run only one unit test

       make unit FILES=test/utils/src/test_num.c

or with the toolbelt

       tb make unit FILES=test/utils/src/test_num.c

## Running unit tests with specific build settings

Defines are managed by make and are passed on to the unit test code. Use the
normal ways of configuring make when running tests. For instance to run test
for Crazyflie 1

      make unit LPS_TDOA_ENABLE=1

## Dependencies

Frameworks for unit testing and mocking are pulled in as git submodules.

The testing framework uses ruby and rake to generate and run code.

To minimize the need for installations and configuration, use the docker builder
image (bitcraze/builder) that contains all tools needed. All scripts in the
tools/build directory are intended to be run in the image. The
[toolbelt](https://wiki.bitcraze.io/projects:dockerbuilderimage:index) makes it
easy to run the tool scripts.
