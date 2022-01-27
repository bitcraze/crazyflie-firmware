---
title: On-chip debugging
page_id: openocd_gdb_debugging
---

One of the key components of getting serious about developing on
the Crazyflie, is to dive into the C-based firmware. If you want
to do some major changes to the intrinsics of the code, it is essential
to have proper tools to be able to debug the code (place breakpoints,
read real-time values etc\...). On this page, you can find examples on
how debug the STM32 from VS Code and Eclipse. Debugging the nRF51 chip requires a [different configuration](https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/development/starting_development/), but an otherwise identical set-up.

> **_NOTE:_**
> Debugging requires our [debug adapter](https://www.bitcraze.io/products/debug-adapter-kit/) and an ST Link V2 Debugger or similar.

## Debugging in VS Code

### Prerequisites

First ensure that you have the ARM GCC toolchain and OpenOCD installed and in your path. To check, run:

    which openocd
    which arm-none-eabi-gcc

The path to your OpenOCD binary and ARM GCC binary should output. If not, try installing them again.

#### Ubuntu

These steps have been tested on Ubuntu 20.04. The link to gdb-multiarch is required because Ubuntu does not ship arm-none-eabi-gdb anymore, but the new gdb-multiarch that supports all architecture.

    sudo apt-get install openocd
    sudo apt-get install gcc-arm-none-eabi gdb-multiarch
    sudo ln -s /usr/bin/gdb-multiarch /usr/local/bin/arm-none-eabi-gdb

If you do not have vscode yet, the easiest way to install it on Ubuntu is via snap using 'Ubuntu Software' of by typing:

    sudo snap install --classic code


#### Mac OS

    brew install open-ocd
    brew tap PX4/homebrew-px
    brew install arm-none-eabi-gcc

### The Cortex Debug Extension

Install the [extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) either by clicking "Install" on the web page, or by searching "Cortex Debug" in the Extensions tab of VS Code.

Click on "Run", then "Add Configuration", then "Cortex Debug".

![VS Code add configuration](/docs/images/vscode_add_configuration.webp)

This should automatically create the needed "launch.json" file.

#### Cortex Debug Configuration

Inside of the file, replace everything with the following:

    {
        // Use IntelliSense to learn about possible attributes.
        // Hover to view descriptions of existing attributes.
        // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
        "version": "0.2.0",
        "configurations": [
            {
                "name": "STM32 Debug",
                "cwd": "${workspaceRoot}",
                "executable": "./cf2.elf",
                "request": "launch",
                "type": "cortex-debug",
                "device": "STM32F405",
                "svdFile": "STM32F405.svd",
                "servertype": "openocd",
                "configFiles": ["interface/stlink-v2.cfg", "target/stm32f4x.cfg"],
                "runToMain": true,
                "preLaunchCommands": [
                    "set mem inaccessible-by-default off",
                    "enable breakpoint",
                    "monitor reset"
                ]
            },
            // {
            //     "name": "STM32 App Debug"
            // }
        ]
    }

- "svdFile" refers to the necessary file for peripheral registers to show up nicely in the debug pane, all named and structured; we'll add it in the next step
- "configFiles" refers to the files you need so that OpenOCD knows what device and interface you're using; it should already come with them
- "device" refers to the device you want to debug
- "servertype" refers to the debugging server to use. We recommend OpenOCD, but other servers can be used, if supported by the Cortex Debug extension.
- "runToMain" tells the GDB debug server to jump to main by default
- "preLaunchCommands" specifies the commands for the GDB server to send before giving away control to you; the commands here mimic the options that the tutorial for Eclipse below specifies

> **_NOTE:_**
> To debug an app, make sure that "cwd" points to the root dir of your app. Note that the "svdFile" path is relative to the "cwd" dir. You can add your app debugger as a separate configuration.

#### Installing the SVD file

Now for the SVD file: just download it from [here](https://raw.githubusercontent.com/posborne/cmsis-svd/master/data/STMicro/STM32F405.svd) and into the firmware root dir. Make sure it has the exact name of "STM32F405.svd"!

### Debug!

After setup, go to the 'Run and Debug' tab of VS Code (on the left sidebar, the icon with the little bug next to the play button), select the chip you want to debug from the drop-down menu at the top of the pane, and hit the play button!

> **_NOTE:_**
> Make sure your executable (cf2.elf) is identical to the one running on your Crazyflie.

If you followed everything, it should start running nicely and look a little something like the image below. Notice the nice peripherals pane at the bottom, along with the variables pane at the top. Awesome, now you can code _and_ debug all within VS Code!

![VS Code Cortex Debug](/docs/images/vscode_cortex_debug.webp)

---

## Debugging in Eclipse

### Ubuntu

> **_Versions:_**
>
> - Ubuntu 18.04.2 LTS (64 bit)
> - Eclipse 2019-03 (Eclipse IDE for C/C++ Developers)

#### Installing prerequisites

First install GDB and openOCD:

    sudo apt-get update
    sudo apt-get install gdb
    sudo apt-get install openocd

Then install java:

    sudo apt install default-jre

Then install Eclipse itself: Go to their download page: [Eclipse
20](https://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/2019-03/R/eclipse-cpp-2019-03-R-linux-gtk-x86_64.tar.gz)
and then go into you download folder and unzip the file.

    tar -zxvf "your-downloaded-file".tar.gz

and start up Eclipse:

    "YOUR-UNZIPPED-FOLDER"/.eclipse

### Mac OS

Install gdb and openocd

    brew install gdb
    brew install open-ocd

Install Java JDK [java
download](https://www.oracle.com/technetwork/java/javase/downloads/index.html)

Download Eclipse [Eclipse
download](https://www.eclipse.org/downloads/download.php?file=/oomph/epp/2019-06/R/eclipse-inst-mac64.dmg)

Choose destination folders - Install

Run Eclipse and choose work folder

##### Installing required Eclipse plugins

The rest is the same as for Linux. Make sure that the arm-none-eabi-gcc
is properly installed and its path is configured in the _debug
configurations_.

### Installing required Eclipse Plugins

Install the C++ development tools and GNU MCU plugin by following the
instructions [here](https://gnu-mcu-eclipse.github.io/plugins/install/).

- C++ Development - Follow the instructions under the header \'CDT\'.
- GNU MCU plugin - Follow the instructions under the header \'Plug-ins
  install/update -\> The Eclipse Marketplace way\'

### Import Crazyflie Firmware

First import the
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)
into Eclipse:

- File > import...
- C/C++ > Existing Code as Makefile Project -> Next
- Give it a name
- Existing Code Location > Browse... > //Look for the firmware folder//
- //Toolchain for Indexer Settings// can be ignored.
- Finish

### Setting up Eclipse Debugging environment

- Go to: Run \> Debug Configurations\...
- Double click \'GDB OpenOCD Debugging\'

Now input the following settings in the debug configurations:

#### Main

![stm openocd main](/docs/images/stm_openocd_main.png)

Insert the filepath to the cf2.elf file to _C/C++ Application_.

#### Debugger

![stm openocd debug](/docs/images/stm_openocd_debugger.png)

check the following settings: OpenOCD setup -\> Config options: \<code\>
-f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c targets
\</code\> GDB Client Setup: 

- Executable name: Filepath to gdb toolchain
- Commands: \<code\> set mem inaccessible-by-default off \</code\>

#### Startup

![stm openocd startup](/docs/images/stm_openocd_startup.png)

### Debug!

If you don\'t see any errors, Eclipse should go to an dedicated
debugging environment automatically and it automatically halts the
crazyflie\'s firmware before it goes into the main function of
src/init/main.c. Press F8 or Run \> Resume to let it continue and place
a breakpoints anywhere in the code (double clicking just in front of the
line in the gray area or press Shift+Ctrl+B). Now you should be able to
read out the values of the defined variables at that very position.

> **_NOTE:_**
> Make sure your executable (cf2.elf) is identical to the one running on your Crazyflie.