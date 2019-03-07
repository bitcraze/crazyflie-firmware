#!/usr/bin/env python3
# Scripts acquiring the base-stations position from a running SteamVR system
# At least one object, HDM controller or tracker, should be tracked in order
# for the script to work.
#
# The output can be directly copy-pasted in the Crazyflie firmware lighthouse.c
# lighthouse deck driver.

import sys
import openvr

CENTER_AROUND_CONTROLLER = False

print("Openning OpenVR")
vr = openvr.init(openvr.VRApplication_Other)

print("OpenVR Oppened")
devices = {}
poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                           openvr.k_unMaxTrackedDeviceCount)

if CENTER_AROUND_CONTROLLER:
    offset = None
    # Acquire offset
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if poses[i].bPoseIsValid:
            device_class = vr.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller or \
               device_class == openvr.TrackedDeviceClass_GenericTracker:
                pose = poses[i].mDeviceToAbsoluteTracking
                offset = [pose[0][3], pose[1][3], pose[2][3]]
                break

    if offset is None:
        print("Controller not found, place controller at the origin of the space")
        openvr.shutdown()
        sys.exit(1)
else:
    offset = [0, 0, 0]

print("Origin: {}", offset)


print("-------------------------------")

for i in range(openvr.k_unMaxTrackedDeviceCount):
    if poses[i].bPoseIsValid:
        device_class = vr.getTrackedDeviceClass(i)
        if (device_class == openvr.TrackedDeviceClass_TrackingReference):
            pose = poses[i].mDeviceToAbsoluteTracking

            position = [pose[0][3] - offset[0], pose[1][3] - offset[1], pose[2][3] - offset[2]]
            rotation = [pose[0][:3], pose[1][:3], pose[2][:3]]

            print("{.origin = {", end='')
            for i in position:
                print("{:0.6f}, ".format(i), end='')
            
            print("}, .mat = {", end='')
            
            for i in rotation:
                print("{", end='')
                for j in i:
                    print("{:0.6f}, ".format(j), end='')
                print("}, ", end='')
            
            print("}},")

openvr.shutdown()
