---
title: Lighthouse limitations
page_id: lh_limitations
---

The lighthouse deck is released in early access which means that there is a couple of limitation you should be aware of:

* There are currently two ways to configure the base station positions/geometry.
  1. Hard-code the base station position in the firmware. This means that you need to re-flash your Crazyflies each time you move the basestations.
  2. Uploded from a computer. The positions can be uploaded to the Crzyflie from a computer, but will be gone when the Crazyflie is restarted. If the ```get_bs_geometry.py``` script is executed with the ```--write``` flag, the base station positions are uploded to the Crazyflie, which can be convenient to test a system.
* Position/gemometry handling will be improved in the future.
* Since the deck only has horizontal sensors, the angle at which the base-stations are seen cannot be too shallow. This means that you should fly at least 40cm bellow the base-stations and that the base-stations should be placed above the flight space. This is a hardware limitation of the current deck.
* The Crazyflie currently only supports Vive base station V1. Base station V1 are limited to two base-station per system. Base-station V2 does not have this limitation and so allows to cover much larger space. Support for the base station V2 is currently being worked-on and should be available in a future firmware update.

## Experimental base station V2 support

There is limited and experimenatal support for V2 base stations.

* 1 and 2 base stations are supported
* The base stations must be configured to use channel 1 and 2
* Calibration data is not read from the base stations and there might be fairly large errors in the angle calculations. For this reason it is likely that the crossing beam positioning method will work better than the default sweep method when using 2 base stations. Change by setting the ```lighthouse.method``` parameter to ```0```.
