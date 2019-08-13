---
title: Development for STM32
page_id: starting_development 
---

This page aims at documenting how to start developing with Crazyflie.
This document should work for the Crazyflie 2.X.


### STM32

Clone the crazyflie-firmware project, or update it using the virtual
machine \"Update all projects\" script. For Crazyflie 2.X make sure the current branch is \"**master**.\"

    ~$ cd projects/crazyflie-firmware/
    crazyflie-firmware$ git checkout master   

Then make the firmware.

For **Crazyflie 2.X**:

```
crazyflie-firmware$ make PLATFORM=cf2
(...)
  DFUse cf2.dfu
Crazyflie 2.0 build!
Build 00:00000000 (20XX.XX.X-XX) CLEAN
Version extracted from git
Crazyloader build!
   text    data     bss     dec     hex filename
  XXXXX    XXXX   XXXXX  XXXXXX   XXXXX cf2.elf
rm version.c
```

To program using the radio bootloader, first install the cflib and cfclient, and put the CF2.X in bootloader mode:


For **Crazyflie 2.X**:
```
    crazyflie-firmware$ make cload
```

From command line the flash make target flashed the firmware using
programming cable
```
    crazyflie-firmware$ make flash
```

#### Command line
From command line the flash make target flashed the firmware using
programming cable

    make flash



