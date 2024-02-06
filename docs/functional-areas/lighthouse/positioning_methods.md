---
title: Lighthouse positioning methods
page_id: lh_positioning_methods
---

There are two basic methods for estimating the position of the Lighthouse deck, they are outlined here.

## Crossing beams

This was the first method to be implemented and is simple and robust, but requires two base stations to be visible.

The idea is to calculate the vectors from two base station to a sensor on the Lighthouse deck. This vector is defined by the
intersection line between the two light planes of the base station and is sometimes referred to as a "beam", hence the name.

In theory the beams should cross in the point where the sensor is located, in real life there are errors and the
beams will not exactly meet. To handle this the algorithm calculates the point that is closest to both beams instead, and uses
this as the estimated position.

The distance from the estimated position to the beam is called the delta and is available as a log in the Crazyflie. It provides
a measurement of the error in system.

The calculated position is fed into the Kalman estimator to be used together with other sensor data.

## Raw sweeps

It is preferable to use sensor data that has been processed as lite as possible in a Kalman filter, and that is
what we try to do in this method. The base station geometry and angle of one sweep is passed directly into
the kalman estimator to be used to improve the estimate. The measurement model is based on the fact that the
sensor must be located in the plane that is defined by the base station geometry and sweep angle.

One base station is enough to estimate the position using this method, but more base stations adds precission and redundancy.


## Ground truth

To use the lighthouse positioning as a ground truth measurement for your research, you should enable the 'lighthouse positioning system as groundtruth' in the [kbuild Expansion deck configuration](/docs/development/kbuild.md).
 This will default the position estimator for lighthouse to be crossing beam (which you should not change), and you will be able to get the X, Y, Z position from the logs ```lighthouse.x/.y/.z```
