#!/usr/bin/env python3
# Scripts acquiring the base-stations position from a running SteamVR system
# At least one object, HDM controller or tracker, should be tracked in order
# for the script to work.
#
# The output can be directly copy-pasted in the Crazyflie firmware lighthouse.c
# lighthouse deck driver.

import sys
import openvr
import numpy as np

CENTER_AROUND_CONTROLLER = False

print("Opening OpenVR")
vr = openvr.init(openvr.VRApplication_Other)

print("OpenVR Opened")
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

bs_poses = [None, None]

for i in range(openvr.k_unMaxTrackedDeviceCount):
    if poses[i].bPoseIsValid:
        device_class = vr.getTrackedDeviceClass(i)
        if (device_class == openvr.TrackedDeviceClass_TrackingReference):

            mode = vr.getStringTrackedDeviceProperty(i, openvr.Prop_ModeLabel_String)
            try:
                mode = mode.decode("utf-8")
            except:
                #likely already decoded
                pass

            pose = poses[i].mDeviceToAbsoluteTracking

            # Mode 'B' is master
            if mode == 'B':
                bs_poses[0] = pose
            elif mode == 'A' or mode == 'C':
                bs_poses[1] = pose
            else:
                print("Base station with mode {} detected.".format(mode))
                print("This script can only work with base station V1 (mode A, B or C). Exiting.")
                sys.exit(1)

# Rotation matrixes to convert to the CF coordinate system
opencv_to_cf = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

cf_to_opencv = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
])


for pose in bs_poses:
    if pose is None:
        continue

    position = [pose[0][3] - offset[0], pose[1][3] - offset[1], pose[2][3] - offset[2]]
    position_cf = np.dot(opencv_to_cf, position)

    rotation = [pose[0][:3], pose[1][:3], pose[2][:3]]
    rotation_cf = np.dot(opencv_to_cf, np.dot(rotation, cf_to_opencv))

    print("{.origin = {", end='')
    for i in position_cf:
        print("{:0.6f}, ".format(i), end='')

    print("}, .mat = {", end='')

    for i in rotation_cf:
        print("{", end='')
        for j in i:
            print("{:0.6f}, ".format(j), end='')
        print("}, ", end='')

    print("}},")

openvr.shutdown()
