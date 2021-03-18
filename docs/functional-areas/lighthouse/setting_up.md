---
title: Setting up the lighthouse system
page_id: lh_setting_up
---

## Prerequisites

The lighthouse deck allows to use the HTC-Vive/SteamVR lighthouse tracking system to fly Crazyflies autonomously. The system works with one or two Lighthouse base stations (two recommended), both V1 and V2 are supported. It is not possible to mix V1 and V2 base stations in the same system.

After everything is setup, the computer is not required anymore: the Crazyflie will autonomously estimate its position from the lighthouse signals.

## The basics

The information obtained from a lighthouse system is essentially two angles describing the direction from the base station to each light sensor on the lighthouse deck. All measurements and position calculations are performed in the Crazyflie.

The Crayzflie needs two pieces of system information to be able to estimate its position. The first is calibration data for all base stations in the system, the second is geometry data.

The calibration data describes slight imperfections in the the manufacturing of the base stations. The calibration data is measured in the factory and is stored in each base station. The calibration data is required to calculate the angles with a high level of accuracy.

The geometry data describes the position and orientation of the base stations and is needed to understand how the measured angles relate to the global reference frame.

Calibration and geometry data is stored in the Crazyflie to make it possible to take off and use the lighthouse system imediate after reboot.

The Crazyflie must know if the system is using V1 or V2 type base stations, this is set manually from the client.

## Set up procedure

The following procedure assumes two base stations are used, but the instructions for one should be similar.

You need the python client to complete the procedure.

### 1. Configure the base stations

For lighthouse V1 set up the base stations according to the HTC instructions, depending if you want to use a sync cable or not.

For lighthouse V2 the base station should be configured to use channel 1 and 2, this can be done through the client:

1. Connect the first base station (only one of them) to your computer via USB
1. Open the lighthouse tab in the client
1. Click the "Set BS channel" button
1. In the dialog box click the "Scan basestation" button
1. Chose channel 1
1. Click the "Set channel" button
1. Repeat for the other base station but configure it for channel 2

It is now time to mount the base stations in your space.

## 2. Update Crazyflie and lighthouse deck firmware

1. In the client connect to the Crazyflie using a Crazyradio (Note: USB will not work)
1. Click "Bootloader" in the "Connect" menu
1. In the dialog box, in the drop down chose the latest release (the default)
1. Click the "Program" button.

The Crazyflie and the deck will be flashed with the latest firmware.

Note 1: the Crazyflie will restart one or more times during the flashing process.

Note 2: during the transition into the first release of the lighthouse system the default release may not contain a lighthouse deck binary.
If this is the case, please download it manually from [github](https://github.com/bitcraze/crazyflie-release/releases) and use the
file option in the bootloader dialog.
## 3. Obtaining calibration data

The calibration data is transmitted in the light from the base stations. The Crazyflie will receive the data and store it
when it is complete. After startup the latest stored data will automatically be loaded from memory.

The procedure is:

1. Turn on the Crazyflie and put it on the floor with both base stations within range
1. Start the python client and connect to the Crazyflie
1. Open the lighthouse tab
1. The "Receiving" indicators should be green, showing that both base stations are received
1. Wait until both "Calibration" indicators turn green

The calibration data has now been saved in the Crazyflie.

It may take up to 30 seconds to reveive the calibration data, especially for lithghthouse V2 the process may take a while
and can be sensitive to interference. If the calibration data is not received for both base stations, try to hold the
Crazyflie closer to one of the base stations at a time to get a better signal.

## 4. Geometry data

The base station positions and orientation can be measured using a Crazyflie + lighthouse deck. The Crazyflie **must** have the correct calibration data first to get a useful result.

1. Put the Crazyflie on the floor where you want the origin. Forward of the Crazyflie is the positive X-axis while
the Y-axis points to the left. Z is up.
1. In the lighthouse tab in the python client, make sure the calibration data has been received
1. Click the "Manage geometry" button
1. In the dialog box click the "Estimate Geometry" button
1. After a while the estimated position of the base stations shoule be displayed as a list
1. If the positions of the base stations (relative to the Crazyflie) looks reasonable click the "Write to Crazyflie" button

The geometry data has now been saved in the Crazyflie and the system is ready to be used.

The 3D visualization should show the two base stations and the estimated position of the Crazyflie.

**Note1:** Make sure the Lighthouse deck is parallell to the floor, as the XY-plane of the coordinate system will be an extension of the deck.


## The number of base stations and frame synchronization
The lighthouse deck works with one or two basestations but the estimated position will be better and more stable with two basestations. When using two basestations, one of them may be occluded temporarily, and the Crazyflie will use the other one for positioning.

The protocol for the lighthouse V1 is composed of frames starting with sync pulses from the basestations. The sync pulses are used to identify which basestation the frame is originating from and this information is essential for correct positioning. When one basestation is occluded, only sync pulses from the visible basestation will be available to the system, which is fine as long as the the system can keep track of the frames. If we loose track of the frames, for instance if both basestations are occluded, the system has to re-synch again to function, but this is a quick process when the basestations are visible. Due to the design of the lighthouse protocol, visibility to both basestations is always required for synchronization or re-synchronization in a two basestation system.

The protocol in lighthouse 2 does not use the same frame concept and there is no need for frame sync.

The lighthouse V2 protocol supports more than 2 base stations and most of the Crazyflie firmware is designed for this as well but the tools currently only support two base stations.
The ```PULSE_PROCESSOR_N_BASE_STATIONS``` (in pulse_processor.h) determines the number of base stations that are handled by the system and can be increased by brave users. This feature is very much untested and there is currently no support to estimate the geometry of a 2+ system.

## Position estimation methods
There are currently two ways of calculating the position using the lighthouse.

The first method that was implemented calculates two vectors (or beams) from the basestations, based on the sweep angles measured by the sensors on the Crazyflie. The intersection point between the beams is the position of the Crazyflie, and this point is pushed into the estimator. We call this method the “crossing beam” method. A more recent implementation pushes the sweep angles from the base stations into the estimator and lets the kalman filter calculate the position based on this information. This method is called the “sweep angle” method and also works if only one basestation is available. It is possible to change positioning method “on the fly” by setting the lighthouse.method parameter. The sweep angle method is the default.
