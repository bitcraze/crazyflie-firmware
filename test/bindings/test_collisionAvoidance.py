import sys

import numpy as np
import pytest

from pycrazyswarm import *
from pycrazyswarm.util import *


RADII = np.array([0.125, 0.125, 0.375])
Z = 1.0


def setUp(args):
    crazyflies_yaml = """
    crazyflies:
    - id: 1
      channel: 100
      initialPosition: [-1.0, 0.0, 0.0]
    - id: 2
      channel: 100
      initialPosition: [1.0, 0.0, 0.0]
    """
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args=args)
    timeHelper = swarm.timeHelper
    return swarm.allcfs, timeHelper


def test_velocityMode_sidestepWorstCase(args=None):
    if args is None:
        args = "--sim --vis null --dt 0.05 --maxvel 1.0"

    allcfs, timeHelper = setUp(args)
    a, b = allcfs.crazyflies

    a.enableCollisionAvoidance([b], RADII)
    b.enableCollisionAvoidance([a], RADII)

    a.cmdVelocityWorld([1.0, 0.0, 0.0], yawRate=0.0)
    b.cmdVelocityWorld([-1.0, 0.0, 0.0], yawRate=0.0)

    while timeHelper.time() < 10.0:
        positions = np.stack([a.position(), b.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        assert not np.any(collisions)

        timeHelper.sleep(timeHelper.dt)
        if a.position()[0] > 1.0 and b.position()[0] < -1.0:
            return
    assert False


def test_goToWithoutCA_CheckCollision():
    args = "--sim --vis null"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies

    cf0.goTo([1.0, 0.0, 1.0], 0, 5.0)
    cf1.goTo([-1.0, 0.0, 1.0], 0, 5.0)
    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        if np.any(collisions):
            return
        timeHelper.sleep(timeHelper.dt)
    assert False


def test_goToWithCA_CheckCollision():
    args = "--sim --vis null"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    cf0.enableCollisionAvoidance([cf1], RADII)
    cf1.enableCollisionAvoidance([cf0], RADII)

    cf0.goTo([1.0, 0.0, 1.0], 0, 5.0)
    cf1.goTo([-1.0, 0.0, 1.0], 0, 5.0)
    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        assert not np.any(collisions)
        timeHelper.sleep(timeHelper.dt)


def test_goToWithCA_CheckDestination():
    args = "--sim --vis null"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    cf0.enableCollisionAvoidance([cf1], RADII)
    cf1.enableCollisionAvoidance([cf0], RADII)

    goal0 = [1.0, 0.0, 1.0]
    goal1 = [-1.0, 0.0, 1.0]
    cf0.goTo(goal0, 0, 5.0)
    cf1.goTo(goal1, 0, 5.0)
    timeHelper.sleep(5)
    assert np.all(np.isclose(cf0.position(), goal0))
    assert np.all(np.isclose(cf1.position(), goal1))


def test_goToWithCA_changeEllipsoid():
    args = "--sim --vis null"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies

    newRADII = np.array([0.1, 0.1, 0.3])
    cf0.enableCollisionAvoidance([cf1], newRADII)
    cf1.enableCollisionAvoidance([cf0], newRADII)

    goal0 = [1.0, 0.0, 1.0]
    goal1 = [-1.0, 0.0, 1.0]
    cf0.goTo(goal0, 0, 5.0)
    cf1.goTo(goal1, 0, 5.0)

    # flag if a collision happened during sleep
    collisionHappend = False
    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisionsOld = check_ellipsoid_collisions(positions, RADII)
        collisionsNew = check_ellipsoid_collisions(positions, newRADII)
        if np.any(collisionsOld):
            collisionHappend = True
        assert not np.any(collisionsNew)

        timeHelper.sleep(timeHelper.dt)
    assert collisionHappend


def test_goToWithCA_Intersection():
    args = "--sim --vis null --dt 0.01"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    cf0.enableCollisionAvoidance([cf1], RADII)
    cf1.enableCollisionAvoidance([cf0], RADII)

    goal0 = [1.0, 1.0, 1.0]
    goal1 = [-1.0, 1.0, 1.0]
    cf0.goTo(goal0, 0, 5.0)
    cf1.goTo(goal1, 0, 5.0)

    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        assert not np.any(collisions)

        timeHelper.sleep(timeHelper.dt)
    assert np.all(np.isclose(cf0.position(), goal0))
    assert np.all(np.isclose(cf1.position(), goal1))


def test_goToWithoutCA_Intersection():
    args = "--sim --vis null --dt 0.05"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies

    goal0 = [1.0, 1.0, 1.0]
    goal1 = [-1.0, 1.0, 1.0]
    cf0.goTo(goal0, 0, 5.0)
    cf1.goTo(goal1, 0, 5.0)

    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        if np.any(collisions):
            return

        timeHelper.sleep(timeHelper.dt)
    assert False


def test_goToWithCA_random():
    rows, cols = 3, 5
    N = rows * cols
    crazyflies_yaml = grid_yaml(rows, cols, spacing=0.5)

    args = "--sim --vis null"
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, args=args)
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    cfs = allcfs.crazyflies

    for _, cf in enumerate(cfs):
        cf.enableCollisionAvoidance(cfs, RADII)

    np.random.seed(0)
    xy_radius = 0.125
    for _ in range(5):
        lastTime = timeHelper.time()
        goals = poisson_disk_sample(N, dim=2, mindist=5*xy_radius)
        goals_z = Z * np.ones(N) + 0.2 * np.random.uniform(-1.0, 1.0, size=N)
        goals = np.hstack([goals, goals_z[:, None]])

        for goal, cf in zip(goals, cfs):
            cf.goTo(goal, yaw=0.0, duration=5)
        while timeHelper.time() - lastTime < 20.0:
            positions = np.stack([cf.position() for cf in cfs])
            timeHelper.sleep(timeHelper.dt)
            collisions = check_ellipsoid_collisions(positions, RADII)
            assert not np.any(collisions)
        for goal, cf in zip(goals, cfs):
            assert np.all(np.isclose(cf.position(), goal, atol=1e-4))


def test_cmdPosition():
    args = "--sim --vis null --maxvel 1.0"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    cf0.enableCollisionAvoidance([cf1], RADII)
    cf1.enableCollisionAvoidance([cf0], RADII)

    goal0 = np.array([1.0, 0.0, 2.0])
    goal1 = np.array([-1.0, 0.0, 2.0])
    cf0.cmdPosition(goal0, yaw=0.0)
    cf1.cmdPosition(goal1, yaw=0.0)
    while timeHelper.time() < 10.0:
        positions = np.stack([cf0.position(), cf1.position()])
        collisions = check_ellipsoid_collisions(positions, RADII)
        assert not np.any(collisions)

        timeHelper.sleep(timeHelper.dt)
    assert np.all(np.isclose(cf0.position(), goal0))
    assert np.all(np.isclose(cf1.position(), goal1))


def test_boundingBox():
    args = "--sim --vis null --maxvel 1.0"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    BBOXMAX = [5, 5, 5]
    BBOXMIN = [-3, -3, -3]
    cf0.enableCollisionAvoidance(
        [cf1], RADII, bboxMax=BBOXMAX, bboxMin=BBOXMIN)
    cf1.enableCollisionAvoidance(
        [cf0], RADII, bboxMax=BBOXMAX, bboxMin=BBOXMIN)

    goal0 = np.array([BBOXMAX[0] - 1, 0.0, Z])
    goal1 = np.array([BBOXMIN[0] - 1, 0.0, Z])
    cf0.cmdPosition(goal0, yaw=0.0)
    cf1.cmdPosition(goal1, yaw=0.0)
    while timeHelper.time() < 10.0:
        assert np.all(cf1.position() >= BBOXMIN)
        assert np.all(cf1.position() <= BBOXMAX)
        timeHelper.sleep(timeHelper.dt)

    assert np.all(np.isclose(cf0.position(), goal0))
    assert not np.all(np.isclose(cf1.position(), goal1))


#@pytest.mark.xfail(reason="Bug in firmware")
def test_maxSpeed_zero():
    args = "--sim --vis null --maxvel 1.0"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies
    cf0.enableCollisionAvoidance([cf1], RADII, maxSpeed=0.0)
    cf1.enableCollisionAvoidance([cf0], RADII, maxSpeed=0.0)

    goal0 = np.array([1.0, 0.0, Z])
    goal1 = np.array([-1.0, 0.0, Z])
    cf0.cmdPosition(goal0, yaw=0.0)
    cf1.cmdPosition(goal1, yaw=0.0)
    while timeHelper.time() < 10.0:
        timeHelper.sleep(timeHelper.dt)
        assert np.all(cf0.velocity() == np.zeros(3))
        assert np.all(cf1.velocity() == np.zeros(3))

    assert np.all(np.isclose(cf0.position(), goal1))
    assert np.all(np.isclose(cf1.position(), goal0))


def test_maxSpeed_limit():
    args = "--sim --vis null --maxvel 1.0"
    allcfs, timeHelper = setUp(args)
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    cf0, cf1 = allcfs.crazyflies

    cf0.enableCollisionAvoidance([cf1], RADII, maxSpeed=1.0)
    cf1.enableCollisionAvoidance([cf0], RADII, maxSpeed=3.0)

    cf0.cmdVelocityWorld([2.0, 0.0, 0.0], yawRate=0.0)
    cf1.cmdVelocityWorld([-2.0, 0.0, 0.0], yawRate=0.0)

    while timeHelper.time() < 10.0:
        assert np.all(np.abs(cf0.velocity()) < 1.2 * np.ones(3))
        assert np.all(np.abs(cf1.velocity()) < 3.2 * np.ones(3))
        timeHelper.sleep(timeHelper.dt)
        if cf0.position()[0] > 2.0 and cf1.position()[0] < -2.0:
            return

    assert False


if __name__ == "__main__":
    # test_velocityMode_sidestepWorstCase(sys.argv)
    test_maxSpeed_limit()
