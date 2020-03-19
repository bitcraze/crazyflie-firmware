---
title: On-chip debugging
page_id: openocd_gdb_debugging
---

One of the key components of getting really serious about developing on
the crazyflie, is to dive into the C-based firmware. If you really want
to do some major changes to the intrinsic of the code, it is essential
to have proper tools to be able to debug the code (place breakpoints,
read real-time values ect\...). On this page, we will put a few examples
how to do this with IDE\'s and different environments.

> **_NOTE:_**
This page requires our debug adapter and ST Link V2 Debugger! See
this page: [Debug adapter](https://wiki.bitcraze.io/projects:crazyflie2:debugadapter:index)


Debugging using eclipse
---
### Ubuntu


> **_Versions:_**
>
>-   Ubuntu 18.04.2 LTS (64 bit)
>-   Eclipse 2019-03 (Eclipse IDE for C/C++ Developers)



#### Installing prerequisites

First install GDB and openOCD:

    sudo apt-get update
    sudo apt-get install gdb
    sudo apt-get install openocd

Then install java:

    sudo apt install default-jre

Then install eclipse itself: Go to their download page: [eclipse
20](https://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/2019-03/R/eclipse-cpp-2019-03-R-linux-gtk-x86_64.tar.gz)
and then go into you download folder and unzip the file.

    tar -zxvf "your-downloaded-file".tar.gz

and start up eclipse:

    "YOUR-UNZIPPED-FOLDER"/.eclipse

#### Installing required Eclipse Plugins

Install the C++ development tools and GNU MCU plugin by following the
instructions [here](https://gnu-mcu-eclipse.github.io/plugins/install/).

-   C++ Development - Follow the instructions under the header \'CDT\'.
-   GNU MCU plugin - Follow the instructions under the header \'Plug-ins
    install/update -\> The Eclipse Marketplace way\'

#### Import Crazyflie Firmware

First import the
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)
into eclipse:

- File > import...
- C/C++ > Existing Code as Makefile Project -> Next
- Give it a name
- Existing Code Location > Browse... > //Look for the firmware folder//
- //Toolchain for Indexer Settings// can be ignored.
- Finish

#### Setting up Eclipse Debugging environment

-   Go to: Run \> Debug Configurations\...
-   Double click \'GDB OpenOCD Debugging\'

Now input the following settings in the debug configurations:
##### Main

![stm openocd main](/images/stm_openocd_main.png)

Insert the filepath to the cf2.elf file to *C/C++ Application*.

##### Debugger

![stm openocd debug](/images/stm_openocd_debugger.png)

check the following settings: OpenOCD setup -\> Config options: \<code\>
-f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c targets
\</code\> GDB Client Setup:

-   Executable name: Filepath to gdb toolchain
-   Commands: \<code\> set mem inaccessible-by-default off \</code\>

##### Startup

![stm openocd startup](/images/stm_openocd_startup.png)

##### Hit Debug!

If you don\'t see any errors, eclipse should go to an dedicated
debugging environment automatically and it automatically halts the
crazyflie\'s firmware before it goes into the main function of
src/init/main.c. Press F8 or Run \> Resume to let it continue and place
a breakpoints anywhere in the code (double clicking just in front of the
line in the gray area or press Shift+Ctrl+B). Now you should be able to
read out the values of the defined variables at that very position.

----
 Make sure that your cf2.elf is the
same as the one you uploaded to the crazyflie!

---

### Mac OS

Install gdb and openocd

    brew install gdb
    brew install open-ocd

Install java JDK [java
download](https://www.oracle.com/technetwork/java/javase/downloads/index.html)

Download eclipse [eclipse
download](https://www.eclipse.org/downloads/download.php?file=/oomph/epp/2019-06/R/eclipse-inst-mac64.dmg)

Choose destination folders - Install

Run eclipse and choose work folder

##### Installing required eclipse Plugins

The rest is the same as for Linux. Make sure that the arm-none-eabi-gcc
is properly installed and its path is configured in the *debug
configurations*.
