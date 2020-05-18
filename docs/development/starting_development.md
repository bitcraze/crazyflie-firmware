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
Build for the CF2 platform!
Build 00:00000000 (20XX.XX.X-XX) CLEAN
Version extracted from git
Crazyloader build!
Flash |  218132/1032192 (21%),  814060 free | text: 213024, data: 5108, ccmdata: 0
RAM   |   71564/131072  (55%),   59508 free | bss: 66456, data: 5108
CCM   |   43528/65536   (66%),   22008 free | ccmbss: 43528, ccmdata: 0
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
