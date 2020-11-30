---
title: Setting up the lighthouse system
page_id: lh_setting_up
---

The lighthouse deck is still in early access and the process of managing system data has not been streamlined yet.

## Prerequisites

The lighthouse deck allows to use the HTC-Vive/SteamVR lighthouse tracking system to fly Crazyflies autonomously. The system works with one or two Lighthouse (V1 or V2) base stations (two recommended). It is not possible to mix V1 and V2 base stations in the same system.

After everything is setup, the computer is not required anymore: the Crazyflie will autonomously estimate its position from the lighthouse signals.

In order to setup the system you must also be able to compile a custom firmware for your Crazyflie and to program your Crazyflie 2.X. To do so you can follow the [Getting started with Crazyflie 2.X](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/) and [Getting started with development](https://www.bitcraze.io/getting-started-with-development/) guides.

## The basics

The information obtained from a lighthouse system is essentially two angles describing the direction from the base station to each light sensor on the lighthouse deck. All measurements and calculations are performed in the Crazyflie.

The Crayzflie needs two pices of system information to be able to use estimate its position. The first is calibration data for all base stations in the system, the second is geometry data.

The calibration data describes slight imperfections in the the manufacturing of the base stations. The calibration data is measured in the factory and is stored in each base station. The calibration data is required to calculate the angles with a high level of accuracy.

The geometry data describes the position and orientation of the base stations and is needed to understand how the measured angles relate to the global reference frame.

## Setting and getting system data

The calibration and geometry data that is used in run time is located in RAM and there are a few methods to set it

1. Store in the permanent storage in the Crazyflie, this is automatically copied to RAM at startup. To write to the permanent storage, data must first be located in RAM.
1. Upload to RAM in runtime.
1. Hard code in the firmware, either in the lighthouse_core.c file or in an app

## Compiling and flashing the firmware

Clone (or download) the [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) repository to your computer.

```
git clone https://github.com/bitcraze/crazyflie-firmware
```

### Enable lighthouse support

To compile the firmware with lighthouse support, use the ```DISABLE_LIGHTHOUSE_DRIVER=0``` compile flag. For instance this can be done by adding ```CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0``` to your config.mk file.

### System identification

The Crayflie will make an educated guess in run time based on the light signals it receives to decide if the system uses lighthouse V1 or V2 base stations. Unfortunately this sometimes fails, and a better option is to comile the firmware with a compile flag to force it to V1 or V2 instead. Add ```CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=1``` or ```CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2``` depending on your system.

### Build and flash

In a terminal window run

```
make clean
make all
make cload
```
See [configure_build.md](/docs/building-and-flashing/configure_build.md) for more information about the config.mk file.

## Python dependencies

A few python scripts are used to handle system data, they require that you have numpy and open cv installed on your computer

```
pip install numpy
pip install opencv-python
```

## Obtaining calibration data

The calibration data is encoded in the light sweeps and is recevied by the Crazyflie when the a base station is within range.
When the Crayzflie has received calibration data there will be a notification in the consol log in the python client.

For lighthouse V1 calibration data is usually received within 5-10 seconds.

In lighthouse 2 reception of the calibration data is a slow process and it is more sensitive to errors and may fail (we hope to improve this), especially with more than one base station. For the lighthouse V2 base stations there is an alternative way to obtain the calibration data via USB. Use the https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/lighthouse/get_lh2_calib_data.py script to get the data. The input to the script is the device, usually something like /dev/tty.usb12345

To read calibration (and geometry data) from RAM in a Crayzflie, use the https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/lighthouse/read_lighthouse_mem.py script.

To write calibration (and geometry data) to RAM in a Crazyflie, use the https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/lighthouse/write_lighthouse_mem.py script.

The https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/lighthouse/persist_bs_data.py script can also be used, it can also write the data to permanent storage which will make it available after re-boot.

**Note**: When there is valid calibration data in RAM for a base station, the data in the light signal is ignored and will not overwrite the current data.

## Geometry data

The base station positions and orientation can be measured using a Crazyflie + lighthouse deck. The Crazyflie **must** have the correct calibration data first to get a useful result.

Use the https://github.com/bitcraze/crazyflie-firmware/blob/master/tools/lighthouse/get_bs_geometry.py script to estimate the base station geometry data. Use the --write flag to upload the data to the Crazyflie RAM.

**Note1:** Make sure the Lighthouse deck is parallell to the floor, as the XY-plane of the coordinate system will be an extension of the deck.

**Note2:** Always check the output of the script to make sure that the position (.origin) components of the result have reasonable values.

## The setup process

The current recommended process to set up a lighthouse system is slightly different between V1 and V2.

### Lighthouse V1

1. Make sure the base stations are mounted at apropriate positions, see the HTC Vive documentation.
1. Build and flash the firmware with lighthouse support
1. Start the Crayzflie and put it on the floor, in the origin facing positive X
1. Connect the python client and wait for the consol log to report "Got calibration from XXX" for each base station in the system
1. Run ```tools/lighthouse/get_bs_geometry.py --uri radio://0/80/2M --write``` with the appropriate uri for your Crazyflie
1. The Crazyflie should now have all the data it needs to estimate its position!

The data can be stored in the Crazyflie using the persist_bs_data.py script or by hard coding the values in the lighthouse_core.c file.

### Lighthouse V2

1. Extract calibration data from the base stations, one at a time, by connecting them to a computer via USB and running get_lh2_calib_data.py
1. Make sure the base stations are mounted at apropriate positions, see the Steam VR documentation.
1. Build and flash the firmware with lighthouse support
1. Start the Crayzflie and put it on the floor, in the origin facing positive X
1. Modify the persist_bs_data.py script with the calibration data and run it to store the data in the Crazyflie permanent storage
1. Run ```tools/lighthouse/get_bs_geometry.py --uri radio://0/80/2M --write``` with the appropriate uri for your Crazyflie
1. The Crazyflie should now have all the data it needs to estimate its position!
1. Modify the persist_bs_data.py script with the geometry data and run it to store the data in the Crazyflie permanent storage

## Startup position
The Lighthouse deck has 4 receivers and can recover its orientation. The Crazyflie can be started in any orientation when using the lighthouse system.

You can test if the positioning is working by starting the Crazyflie facing X and using the clients “Position hold” mode, this should drift a little bit (mostly in Z) due to an estimator bug, though you should be able to get a quite stable flight and be able to control the Crazyflie position with the gamepad.

## The number of basestations and frame synchronization
The lighthouse deck works with one or two basestations but the estimated position will be better and more stable with two basestations. When using two basestations, one of them may be occluded temporarily, and the Crazyflie will use the other one for positioning.

The protocol for the lighthouse V1 is composed of frames starting with sync pulses from the basestations. The sync pulses are used to identify which basestation the frame is originating from and this information is essential for correct positioning. When one basestation is occluded, only sync pulses from the visible basestation will be available to the system, which is fine as long as the the system can keep track of the frames. If we loose track of the frames, for instance if both basestations are occluded, the system has to re-synch again to function, but this is a quick process when the basestations are visible. Due to the design of the lighthouse protocol, visibility to both basestations is always required for synchronization or re-synchronization in a two basestation system.

The protocol in lighthouse 2 does not use the same frame concept and there is no need for frame sync.

The lighthouse V2 protocol supports more than 2 base stations and most of the Crazyflie firmware is designed for this as well. The ```PULSE_PROCESSOR_N_BASE_STATIONS``` (in pulse_processor.h) determines the number of base stations that are handled by the system and can be increased by brave users. This feature is very much untested and there is currently no support to estimate the geometry of a 2+ system.

## Position estimation methods
There are currently two ways of calculating the position using the lighthouse.

The first method that was implemented calculates two vectors (or beams) from the basestations, based on the sweep angles measured by the sensors on the Crazyflie. The intersection point between the beams is the position of the Crazyflie, and this point is pushed into the estimator. We call this method the “crossing beam” method. A more recent implementation pushes the sweep angles from the base stations into the estimator and lets the kalman filter calculate the position based on this information. This method is called the “sweep angle” method and also works if only one basestation is available. It is possible to change positioning method “on the fly” by setting the lighthouse.method parameter. The sweep angle method is the default.


## Erasing storage
If there is valid calibration data in the Crazyflie storage, this will be read at start up and used by the system. As long as there is valid calibration data in RAM, no data will be decoded from the base station light sweeps, which this might be a problem in a lighthouse V1 system if you want to use a different set of base stations (that will use different calibration data). The solution is to write invalid data to the crazyflie storage using the persist_bs_data.py script. Modify the script and set ```.valid = False``` in the calibration data and run the script to write it to the Crazyflie. The Crazyflie should now start to decode the calibration data from the base stations again.
