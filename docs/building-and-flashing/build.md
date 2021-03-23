---
title: Building Crazyflie 2.X firmware
page_id: build_instructions
---

## Dependencies

You'll need to use either the [Crazyflie VM](https://github.com/bitcraze/bitcraze-vm),
[the toolbelt](https://github.com/bitcraze/toolbelt) or
install some ARM toolchain.

### Install a toolchain

#### OS X
```bash
brew tap PX4/homebrew-px4
brew install gcc-arm-none-eabi
```

#### Debian/Ubuntu

Tested on Ubuntu 14.04 64b, Ubuntu 16.04 64b, and Ubuntu 18.04 64b:

For Ubuntu 14.04 :

```bash
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
```

For Ubuntu 16.04 and Ubuntu 18.04:

```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt install gcc-arm-embedded
```

Note: Do not use the `gcc-arm-none-eabi` package that is part of the Ubuntu repository as this is outdated.

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

### Out of tree build

By setting the Makefile variable ```CRAZYFLIE_BASE```, it is possible to build the Crazyflie
from outside the project folder. This can be used to implement autonomous behaviour on
top of the Crazyflie firmware by compiling external code in the firmware from an external
repos.

For example, if you have a deck driver file called ```push.c``` that creates a deck driver for
the deck ```bcPush```. You can create a new git repos with ```crazyflie-firmware``` clonned as
submodule and put ```push.c``` in a folder ```src```. The makefile to build a firmware starting
your deck driver automatically will be:

```make
CRAZYFLIE_BASE=crazyflie-firmware

CFLAGS += -DDECK_FORCE=bcPush

VPATH += src/

PROJ_OBJ += push.o

include $(CRAZYFLIE_BASE)/Makefile
```

Note that ```CFLAGS += -DDECK_FORCE=bcPush``` is what would normally be added to ```config.mk```.
Hence, this method also allow to create build configurations folder by building the firmware is a
separate folder with separate configurations.

Both ```tools/make/config.mk``` and ```current_platform.mk``` are sourced from the current folder
and not from the Crazyflie firmware folder.

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
