---
title: The Lighthouse positioning system
page_id: lighthouse_overview
---

The Lighthouse positioning system uses the HTC Vive base stations (aka Lighthouse) togeather with the Lighthouse deck to achieve high precision positioning. The system is functional but still under development and in the Early access phase.

This page contains information on how to work with the deck during development.

## System setup and getting started

### Prerequisites

The lighthouse deck allows to use the HTC-Vive/SteamVR lighthouse tracking system to fly Crazyflies autonomously. The system works with one or two Lighthouse V1 base stations (two recommended).

After everything is setup, the computer is not required anymore: the Crazyflie will autonomously estimate its position from the lighthouse signals.

In order to setup the system you must also be able to compile a custom firmware for your Crazyflie and to program your Crazyflie 2.X. To do so you can follow the [Getting started with Crazyflie 2.X](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/) and [Getting started with development](https://www.bitcraze.io/getting-started-with-development/) guides.

### Finding base station positions - configuring the system

Make sure the base stations are mounted at apropriate positions, see the HTC Vive documentation.

Clone (or download) the [crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) repository to your computer.

```
git clone https://github.com/bitcraze/crazyflie-firmware
```

Make sure you have numpy and open cv installed on your computer

```
pip install numpy
pip install opencv-python
```

Compile the Crazyflie firmware with lighthouse support and flash the Crazyflie. For instance this can be done by adding ```CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0``` to your config.mk file, opening a terminal window and executing

```
make clean
make all
make cload
```
See [configure_build.md](/docs/building-and-flashing/configure_build.md) for more information about the config.mk file.

Mount the Lighthouse deck on the Crazyflie, and place it on the floor where you want your the origin of your coordinate system, and turn it on. The Crazyflie should be oriented in the direction you want the X-axis.

**Note1:** Make sure the Lighthouse deck is parallell to the floor, as the XY-plane of the coordinate system will an extension of the deck.

**Note2:** Wait at least 5 seconds so that the Crazyflie has time to receive calibration data from the Lighthouse base stations.

Run the ```get_bs_geometry.py``` script to measure and calculate the positions of the base stations. Use the appropriate URI for your Crazyflie.

```
tools/lighthouse/get_bs_geometry.py --uri radio://0/80/2M
```

The output from the script should look something like:

        Reading sensor data...
        Connecting to radio://0/80/2M
        Connected to radio://0/80/2M
        Estimating position of base stations...
        {% raw %}{.origin = {0.929796, -1.359615, 3.187089, }, .mat = {{-0.621841, -0.513463, -0.591329, }, {0.451725, -0.851970, 0.264749, }, {-0.639733, -0.102486, 0.761734, }, }},
        {.origin = {0.044224, 3.075050, 3.035368, }, .mat = {{-0.367098, 0.847259, -0.383915, }, {-0.658670, -0.528202, -0.535869, }, {-0.656805, 0.056157, 0.751967, }, }},{% endraw %}


Copy the last two lines and, on the computer or virtual machine you use for Crazyflie development, paste them into the file 'src/modules/src/lighthouse/lighthouse_position_est.c'. They should replace the contents of lighthouseBaseStationsGeometry[].

        baseStationGeometry_t lighthouseBaseStationsGeometry[2]  = {
            {% raw %}{.origin = {0.929796, -1.359615, 3.187089, }, .mat = {{-0.621841, -0.513463, -0.591329, }, {0.451725, -0.851970, 0.264749, }, {-0.639733, -0.102486, 0.761734, }, }},
            {.origin = {0.044224, 3.075050, 3.035368, }, .mat = {{-0.367098, 0.847259, -0.383915, }, {-0.658670, -0.528202, -0.535869, }, {-0.656805, 0.056157, 0.751967, }, }},{% endraw %}
        };

The .origin part represents the position of each base station, while the .mat part is the orientation (rotation matrix). Check that the positions seems reasonable.

Re-compile and flash the firmware again. The Crazyflie is now ready to be used!

It is also possible to use the ```--write``` flag to write the geometry data to the Crazyflie via radio. Please note that the data is not stored permanently in the Crazyflie and will be gone when restarted.

### Startup position
The Lighthouse deck has 4 receivers and can recover its orientation. The Crazyflie can be started in any orientation when using the lighthouse system.

You can test if the positioning is working by starting the Crazyflie facing X and using the clients “Position hold” mode, this should drift a little bit (mostly in Z) due to an estimator bug, though you should be able to get a quite stable flight and be able to control the Crazyflie position with the gamepad.


### The number of basestations and frame synchronization
The lighthouse deck works with one or two V1 basestations but the estimated position will be better and more stable with two basestations. When using two basestations, one of them may be occluded temporarily, and the Crazyflie will use the other one for positioning.

The protocol for the lighthouse is composed of frames starting with sync pulses from the basestations. The sync pulses are used to identify which basestation the frame is originating from and this information is essential for correct positioning. When one basestation is occluded, only sync pulses from the visible basestation will be available to the system, which is fine as long as the the system can keep track of the frames. If we loose track of the frames, for instance if both basestations are occluded, the system has to re-synch again to function, but this is a quick process when the basestations are visible. Due to the design of the lighthouse protocol, visibility to both basestations is always required for synchronization or re-synchronization in a two basestation system.


### Position estimation methods
There are currently two ways of calculating the position using the lighthouse.

The first method that was implemented calculates two vectors (or beams) from the basestations, based on the sweep angles measured by the sensors on the Crazyflie. The intersection point between the beams is the position of the Crazyflie, and this point is pushed into the estimator. We call this method the “crossing beam” method. A more recent implementation pushes the sweep angles from the base stations into the estimator and lets the kalman filter calculate the position based on this information. This method is called the “sweep angle” method and also works if only one basestation is available. It is possible to change positioning method “on the fly” by setting the lighthouse.method parameter. The sweep angle method is the default.

## System limitations

The lighthouse deck is released in early access which means that there is a couple of limitation you should be aware of:

* There are currently two ways to configure the base station positions/geometry.
  1. Hard-code the base station position in the firmware. This means that you need to re-flash your Crazyflies each time you move the basestations.
  2. Uploded from a computer. The positions can be uploaded to the Crzyflie from a computer, but will be gone when the Crazyflie is restarted. If the ```get_bs_geometry.py``` script is executed with the ```--write``` flag, the base station positions are uploded to the Crazyflie, which can be convenient to test a system.
* Position/gemometry handling will be improved in the future.
* Since the deck only has horizontal sensors, the angle at which the base-stations are seen cannot be too shallow. This means that you should fly at least 40cm bellow the base-stations and that the base-stations should be placed above the flight space. This is a hardware limitation of the current deck.
* The Crazyflie currently only supports Vive base station V1. Base station V1 are limited to two base-station per system. Base-station V2 does not have this limitation and so allows to cover much larger space. Support for the base station V2 is currently being worked-on and should be available in a future firmware update.

### Experimental base station V2 support

There is limited and experimenatal support for V2 base stations.

* 1 and 2 base stations are supported
* The base stations must be configured to use channel 1 and 2
* Calibration data is not read from the base stations and there might be fairly large errors in the angle calculations. For this reason it is likely that the crossing beam positioning method will work better than the default sweep method when using 2 base stations. Change by setting the ```lighthouse.method``` parameter to ```0```.
