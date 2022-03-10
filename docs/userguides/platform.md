---
title: Platforms
page_id: platform
---

The platform concept is a mechanism that exists to make it possible to build flavours of the Crazyflie
firmware that fits hardware platforms with different capabilities and properties.

A platform is identified by a short name that is used through out the code base, in build tools as well as in file names.
Each platform implements the functionality on one or more hardware device types, usually devices with similar properties,
for instance Crazyflie 2.0 and Crazyflie 2.1 both use the `cf2` platform. At compile time, each platform includes the
appropriate drivers, modules and settings and the end result is a binary that can be used on hardware that are
supported by the platform.

It is important to use the correct platform for a specific hardware device. When the device boots, the firmware checks
that it is compatible with the hardware and if it finds that the hardware is not supported it will halt. The reason
the firmware just stops is to avoid any interaction with hardware that might not be compatible, for instance
accidentally start a motor or similar.

The binary files that are built are named based on the platform identifier, for example the binary for the `cf2`
platform is named `cf2.bin`.

## Device type

Each platform supports one or more device types. The device type is identified in runtime and will change some settings
to fit the hardware. The Crazyflie 2.0 and Crazyflie 2.1 are for instance two device types handled by the `cf2` platform,
they will configure the firmware to use different drivers for the IMU based on the device type.

## Persistent paramters
While KBuild is ment for configuring the compile time specific functionality selections the 
[persistent paramters](/docs/userguides/logparam.md#persistent-parameters) are there to configure it in run time. 
And this is the rule of thumb. If it is about a configuration, such as PID tuning or low voltage level, it should 
be a persistent parameter. If it is a functionality it should be a compile time KBuild define. It is not a sharp cut, 
do what is best and easiest for the user.

## Default values

There is functionality to set default values for variables based on the platform and this is for instance used to set
different default tuning values for the (small) Crazyflie 2.1 and the (larger) Crazyflie Bolt. The default values are 
located in e.g. `src/platform/interface/platform_default_cf2.h` for the Crazyflie 2.1.

## Implementation

The main implementation of the platform functionality can be found in `src/platform` and the make system.

An identifier is stored in the physical hardware of each device, which is ued to identify platform and device type.

## Current platforms and devices

|----------|---------------------------------------------|
| Platform | Supported hardware (device type)            |
|----------|---------------------------------------------|
| cf2      | Crazyflie 2.0 (CF20), Crazyflie 2.1 (CF21) |
| bolt     | Crazyflie Bolt 1.0 (CB10)                   |
| tag      | Roadrunner 1.0 (RR10)                       |
|----------|---------------------------------------------|

## Creating your own platform
It is possible to create your own platform! To do so follow the guide in the [development docs](/docs/development/create_platform).
