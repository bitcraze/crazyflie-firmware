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
from numpy import linalg as LA

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

def convertVectorVrToCf(A): #convert a vector from VR perspective format into CF's perspective (front of CF pointing at +X axis)

    def swapRows(A, row1, row2):
        A[[row1, row2]] = A[[row2, row1]] #ref: https://stackoverflow.com/a/54069951/3553367
        return A

    def swapCols(A, col1, col2):
        A[:,[col1,col2]] = A[:,[col2,col1]] #ref: https://stackoverflow.com/a/24507732/3553367
        return A

    def flipCol(A, col):
        A[:,col] *= -1 #ref: https://stackoverflow.com/a/7508703/3553367
        return A

    def flipRow(A, row):
        A[[row]] *= -1 #ref: https://stackoverflow.com/a/7508703/3553367
        return A

    #sequence of swaps and flips determined by observation and trial & error
    #could have multiplied by a rotation vector, not sure
    A = swapRows(A, 0, 2)
    A = swapRows(A, 1, 2)
    A = flipRow(A, 0)
    A = flipRow(A, 1)
    return A


def rotationMatrixBetweenPlanes(a_x, a_y, b_x, b_y):
    # ref: https://math.stackexchange.com/a/1876662/699426
    
    # checks:
    # floating point equality ref: https://www.mathworks.com/help/matlab/ref/eq.html
    # tol = sys.float_info.epsilon; #https://stackoverflow.com/questions/9528421/value-for-epsilon-in-python
    tol = 1e-06

    if abs(LA.norm(a_x) - LA.norm(b_x)) > tol or abs(LA.norm(a_y) - LA.norm(b_y)) > tol:
        raise Exception('Magnitudes mismatch')
    
    if abs( np.dot(a_x, a_y) - np.dot(b_x, b_y)) > tol:
        raise Exception('Dot products mismatch')
    
    if abs( LA.norm(np.cross(a_x, a_y)) - LA.norm(np.cross(b_x, b_y)) ) > tol:
        raise Exception('Magnitude of cross products mismatch')
    
    a_z = np.cross(a_y, a_y) # cannot be np.cross(a_x, a_x)
    b_z = np.cross(b_y, b_y) # cannot be np.cross(b_x, b_x)

    A =  np.transpose(np.array([a_x, a_y, a_z]))
    B =  np.transpose(np.array([b_x, b_y, b_z]))

    R = np.matmul(B, LA.pinv(A))
    
    # example test:
    # B_ = R*A; 
    # B_ == B;
    
    return R

def convertRotationMatrixVrToCf(R_1): #convert a rotation matrix from VR perspective format into CF's perspective (front of CF pointing at +X axis)
    a = [1, 0, 0]
    b = [0, 1, 0]
    c = [0, 0, 1]

    a_r = np.matmul(R_1, a)
    b_r = np.matmul(R_1, b)
    c_r = np.matmul(R_1, c)

    a_r = convertVectorVrToCf(a_r)
    b_r = convertVectorVrToCf(b_r)
    c_r = convertVectorVrToCf(c_r)
    R_2_xy = rotationMatrixBetweenPlanes(a, b, a_r, b_r)
    R_2_yz = rotationMatrixBetweenPlanes(b, c, b_r, c_r)

    R_2 = np.zeros((3, 3))
    R_2[:,0] = R_2_xy[:,0]
    R_2[:,1] = R_2_xy[:,1]
    # R_2[:,1] = R_2_yx[:,1] # equivalent to R_2_xy[:,1]
    R_2[:,2] = R_2_yz[:,2]

    return R_2

for pose in bs_poses:
    if pose is None:
        continue

    position = [pose[0][3] - offset[0], pose[1][3] - offset[1], pose[2][3] - offset[2]]
    position = np.array(position)
    position = convertVectorVrToCf(position)


    rotation = [pose[0][:3], pose[1][:3], pose[2][:3]]
    rotation = np.array(rotation)
    rotation = convertRotationMatrixVrToCf(rotation)

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
