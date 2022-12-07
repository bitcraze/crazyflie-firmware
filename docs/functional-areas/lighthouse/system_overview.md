---
title: Lighthouse system overview
page_id: lh_system_overview
---

The lighthouse deck allows to use the HTC-Vive/SteamVR lighthouse tracking system to fly Crazyflies autonomously. The system works with one or two Lighthouse base stations (two recommended), both V1 and V2 are supported. It is not possible to mix V1 and V2 base stations in the same system.

After everything is setup, the computer is not required anymore: the Crazyflie will autonomously estimate its position from the lighthouse signals.

## The basics

The information obtained from a lighthouse system is essentially two angles describing the direction from the base station to each light sensor on the lighthouse deck. All measurements and position calculations are performed in the Crazyflie.

The Crayzflie needs two pieces of system information to be able to estimate its position. The first is calibration data for all base stations in the system, the second is geometry data.

The calibration data describes slight imperfections in the the manufacturing of the base stations. The calibration data is measured in the factory and is stored in each base station. The calibration data is required to calculate the angles with a high level of accuracy.

The geometry data describes the position and orientation of the base stations and is needed to understand how the measured angles relate to the global reference frame.
The geometry data is calculated by the client using raw angles corrected by calibration data.

Calibration and geometry data is stored in the Crazyflie to make it possible to take off and use the lighthouse system immediate after reboot.

The Crazyflie must know if the system is using V1 or V2 type base stations, this is set manually from the client.

## The number of base stations and frame synchronization
The lighthouse deck works with one or two base stations but the estimated position will be better and more stable with two base stations. When using two base stations, one of them may be occluded temporarily, and the Crazyflie will use the other one for positioning.

The protocol for the lighthouse V1 is composed of frames starting with sync pulses from the base stations. The sync pulses are used to identify which base station the frame is originating from and this information is essential for correct positioning. When one base station is occluded, only sync pulses from the visible base station will be available to the system, which is fine as long as the the system can keep track of the frames. If we loose track of the frames, for instance if both base stations are occluded, the system has to re-synch again to function, but this is a quick process when the base stations are visible. Due to the design of the lighthouse protocol, visibility to both base stations is always required for synchronization or re-synchronization in a two base station system.

The protocol in lighthouse 2 does not use the same frame concept and there is no need for frame sync.

The lighthouse V2 protocol supports more than 2 base stations, up to 4 in the standard firmware. There is currently support implemented for 4+ base stations, for more information please see [the 4+ base station documentation](/docs/functional-areas/lighthouse/multi_base_stations.md).

## Position estimation methods
There are currently two ways of calculating the position using the lighthouse.

The first method that was implemented calculates two vectors (or beams) from the base stations, based on the sweep angles measured by the sensors on the Crazyflie. The intersection point between the beams is the position of the Crazyflie, and this point is pushed into the estimator. We call this method the “crossing beam” method. A more recent implementation pushes the sweep angles from the base stations into the estimator and lets the Kalman filter calculate the position based on this information. This method is called the “sweep angle” method and also works if only one base station is available. It is possible to change positioning method “on the fly” by setting the lighthouse.method parameter. The sweep angle method is the default.

## V1 vs. V2

Crazyflie support two generation of lighthouse base station called V1 and V2.

The main difference between the two versions is that lighthouse V1 is designed to work with 2 base station synchronized either optically or via a cable, and V2 is design to work with any number of independent base stations setup on different channels.
