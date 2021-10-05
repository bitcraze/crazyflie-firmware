#!/usr/bin/env python

import os

import numpy as np
from pycrazyswarm import *
import uav_trajectory

Z = 1.0
FIGURE8_CSV = os.path.dirname(__file__) + "/figure8.csv"

def setUp():
    crazyflies_yaml = """
    crazyflies:
    - channel: 100
      id: 1
      initialPosition: [1.0, 0.0, 0.0]
    - channel: 100
      id: 10
      initialPosition: [0.0, -1.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args="--sim --vis null")
    timeHelper = swarm.timeHelper
    return swarm.allcfs, timeHelper

def _collectRelativePositions(timeHelper, cf, duration):
    t0 = timeHelper.time()
    positions = []
    while timeHelper.time() - t0 < duration:
        positions.append(cf.position() - cf.initialPosition)
        timeHelper.sleep(timeHelper.dt + 1e-6)
    return np.stack(positions)


def test_takeOff():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([0, 0, Z])
        assert np.all(np.isclose(cf.position(), pos, atol=0.0001))

def test_goTo_nonRelative():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([1, 1, Z])
        cf.goTo(pos, 0, 1.0)
    timeHelper.sleep(1.0)

    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([1, 1, Z])
        assert np.all(np.isclose(cf.position(), pos))

def test_goTo_relative():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    allcfs.goTo(np.array([1.0,1.0,1.0]), 0, Z)
    timeHelper.sleep(2.0)

    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([1.0,1.0,2*Z])
        assert np.all(np.isclose(cf.position(), pos))

def test_landing():
    allcfs, timeHelper = setUp()
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)

    for cf in allcfs.crazyflies:
        pos = cf.initialPosition + np.array([0, 0, 0.02])
        assert np.all(np.isclose(cf.position(), pos, atol=0.0001))

def test_uploadTrajectory_timescale():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(FIGURE8_CSV)
    trajId = 100
    cf.uploadTrajectory(trajectoryId=trajId, pieceOffset=0, trajectory=traj)

    # We know the traj isn't close to origin at 3/4 of its duration
    cf.startTrajectory(trajId)
    timeHelper.sleep(0.75 * traj.duration)
    assert np.linalg.norm(cf.position() - cf.initialPosition) >= 0.5

    # Make sure we're back at origin
    timeHelper.sleep(traj.duration)

    # Speeding up time by 2x, we should finish the trajectory in less time
    cf.startTrajectory(trajId, timescale=0.5)
    timeHelper.sleep(0.75 * traj.duration)
    assert np.linalg.norm(cf.position() - cf.initialPosition) <= 0.001

def test_uploadTrajectory_fig8Bounds():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(FIGURE8_CSV)

    trajId = 100
    cf.uploadTrajectory(trajectoryId=trajId, pieceOffset=0, trajectory=traj)
    cf.startTrajectory(trajId)
    positions = _collectRelativePositions(timeHelper, cf, traj.duration)

    # We know the approximate range the bounding box should lie in by plotting
    # the trajectory.
    xs, ys, _ = positions.T
    assert 0.9 < np.amax(xs) < 1.1
    assert -0.9 > np.amin(xs) > -1.1
    assert 0.4 < np.amax(ys) < 0.6
    assert -0.4 > np.amin(ys) > -0.6

def test_uploadTrajectory_reverse():
    allcfs, timeHelper = setUp()
    cf = allcfs.crazyflies[0]

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(FIGURE8_CSV)
    trajId = 100
    cf.uploadTrajectory(trajectoryId=trajId, pieceOffset=0, trajectory=traj)

    cf.startTrajectory(trajId)
    positions = _collectRelativePositions(timeHelper, cf, traj.duration)

    cf.startTrajectory(trajId, reverse=True)
    positionsReverse = _collectRelativePositions(timeHelper, cf, traj.duration)
    positions2 = np.flipud(positionsReverse)

    # The distance threshold must be large because the trajectory is not
    # symmetrical, not because time/reversing is super sloppy.
    dists = np.linalg.norm(positions - positions2, axis=1)
    assert not np.any(dists > 0.2)

def test_uploadTrajectory_broadcast():
    allcfs, timeHelper = setUp()
    cf0, cf1 = allcfs.crazyflies

    relativeInitial = cf1.initialPosition - cf0.initialPosition

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(FIGURE8_CSV)
    trajId = 100
    for cf in (cf0, cf1):
        cf.uploadTrajectory(trajectoryId=trajId, pieceOffset=0, trajectory=traj)

    allcfs.startTrajectory(trajId)
    t0 = timeHelper.time()
    while timeHelper.time() - t0 < traj.duration:
        relative = cf1.position() - cf0.position()
        assert np.all(np.isclose(relativeInitial, relative))
        timeHelper.sleep(timeHelper.dt + 1e-6)

def test_setGroupMask():
    allcfs, timeHelper = setUp()
    cf0, cf1 = allcfs.crazyflies
    cf0.setGroupMask(1)
    cf1.setGroupMask(2)
    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 1)
    timeHelper.sleep(1.5+Z)
    
    pos0 = cf0.initialPosition + np.array([0, 0, Z])
    assert np.all(np.isclose(cf0.position(), pos0, atol=0.0001)) 
    assert np.all(np.isclose(cf1.position(), cf1.initialPosition, atol=0.0001)) 

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 2)
    timeHelper.sleep(1.5+Z)

    pos1 = cf1.initialPosition + np.array([0, 0, Z])
    assert np.all(np.isclose(cf0.position(), pos0, atol=0.0001)) 
    assert np.all(np.isclose(cf1.position(), pos1, atol=0.0001)) 
