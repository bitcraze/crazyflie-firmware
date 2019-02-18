# Crazyflie Firmware  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including
the Crazyflie 2.X and the Roadrunner.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag. 

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

Tested on Ubuntu 14.04 64b and Ubuntu 16.04 64b:

For ubuntu 14.04 :

```bash
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
```

For ubuntu 16.04 :

```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
```

After adding the corresponding repositories, execute the following commands

```bash
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
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
CLOAD=0
```
More information can be found on the 
[Bitcraze wiki](http://wiki.bitcraze.se/projects:crazyflie2:index)

## Folder description:
```
./              | Root, contains the Makefile
 + init         | Contains the main.c
 + config       | Configuration files
 + drivers      | Hardware driver layer
 |  + src       | Drivers source code
 |  + interface | Drivers header files. Interface to the HAL layer
 + hal          | Hardware abstraction layer
 |  + src       | HAL source code
 |  + interface | HAL header files. Interface with the other parts of the program
 + modules      | Firmware operating code and headers
 |  + src       | Firmware tasks source code and main.c
 |  + interface | Operating headers. Configure the firmware environment
 + utils        | Utils code. Implement utility block like the console.
 |  + src       | Utils source code
 |  + interface | Utils header files. Interface with the other parts of the program
 + platform     | Platform specific files. Not really used yet
 + tools        | Misc. scripts for LD, OpenOCD, make, version control, ...
 |              | *** The two following folders contains the unmodified files ***
 + lib          | Libraries
 |  + FreeRTOS  | Source FreeRTOS folder. Cleaned up from the useless files
 |  + STM32...  | Library folders of the ST STM32 peripheral libs
 |  + CMSIS     | Core abstraction layer
```
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
