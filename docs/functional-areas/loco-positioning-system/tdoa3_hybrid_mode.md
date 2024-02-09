---
title: Loco TDoA3 hybrid mode
page_id: loco_tdoa3_hybrid_mode
---

TDoA3 Hybrid mode is an experimental extension to the standard TDoA3 positioning mode. In standard TDoA3 the Crazyflies
are passively listening to the UWB traffic between anchors for positioning, while the hybrid mode also enables the
Crayzflies to transmit UWB packets. This makes it possible to use TWR for positioning the Crazyflie and use a Crazyflie
as an anchor.

## Functionality

In a TDoA3 system the anchors are doing TWR with each other and the TDoA positioning that is used by the Crazyflies
is almost a "side effect". For positioning to work the anchors are also including their own position in the transmitted
packets, see [the TDoA3 protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa3_protocol/).

The TDoa3 implementation is designed to be dynamic in terms of availability of anchors and if an anchor is added or
removed the system will adopt to the new environment, similarly if a Crazyflie passes through a large system it will
adopt to the availability of anchors that are reachable in the current location. As long as each anchor uses a unique
id all parts of the system will be able to handle dynamic situations.
This property makes it possible to convert a passively listening Crazyflie to an actively transmitting node, the other
members of the network will simply see it as another anchor and respond accordingly.

## Use cases

The most basic use case is to let a Crazyflie temporarily do TWR to measure the absolute distance to anchors instead of
the relative distance measurements used in TDoA. TWR is more stable than TDoA when flying outside the convex hull of the
anchors and it enables better positioning.

If two Crazyflies turns on TWR it is possible to measure the distance between the two Crazyflies which could be used
for collision avoidance or relative positioning for instance.

Note that it is possible to use TWR and TDoA at the same time for estimating the position, the Crazyflie receives all
packets and will use them for TDoA and/or TWR depending on if the remote party has received a packet from the Crazyflie
that can be used for TWR.

If a Crazyflie includes its position in the transmitted packets, other Crazyflies can also use the UWB traffic for TDoA
or TWR positioning, just like another anchor. This makes it possible for a Crazyflie to fly to a new position, land and act
as an anchor for other Crazyflies to use for positioning and thus extend the reach of the positioning system dynamically.
There is an option to samples the current estimated position when the Crazyflie starts to transmit its position, the
sampled position will be used in all transmissions to avoid a dynamically changing position.

It is also possible to use the currently estimated position in the transmissions to use a Crazyflie as moving anchor.
It might work for one moving anchor, but most likely the system will be instable if many Crazyflies do this at the
same time without extended error handling as they will use each others position estimates (including errors) to
estimate their own position. An extended error handling is not included in the implementation but hopefully it can be
used as a base if someone is interested in experimenting in this area.

## Implementation

The hybrid mode is mainly implemented in the `lpsTdoa3Tag.c` file and is normally not compiled into the firmware, to
enable it use the `CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE` kbuild flag (`Expansion deck configuration/Support TDoA3 hybrid mode`
in `menuconfig`).

There are no changes or extensions to the UWB protocol and the standard TDoA3 protocol is used. There are also no special
handling in relation to position estimation, the same measurement models in the kalman estimator are used as in standard
TDoA and TWR.

## Params and logs

There are a few parameters that must be configured to use the hybrid mode functionality, they are all part of the
`tdoa3` parameter group.

| Parameter     | Functionality                                                                                        |
|---------------|------------------------------------------------------------------------------------------------------|
| `hmId`        | The anchor id used in transmissions, this id must be unique in the system. Don't use 255 as it means "broadcast" and is received by all nodes |
| `hmTdoa`      | Indicates if received packets should be used for TDoA positioning                                    |
| `hmTwr`       | Indicates if TWR packets should be transmitted                                                       |
| `hmTwrTXPos`  | Indicates if a position should be included in transmitted packets, that is enable other Crazyflies to use the transmissions for TDoA positioning |
| `hmTwrEstPos` | Indicates if received packets should be used for TWR positioning, requires hmTwr to be set           |
| `twrStd`      | The measurement noise to use when sending TWR measurements to the estimator                          |
| `hmAnchLog`   | Select an anchor id to use for logging distance to a specific anchor. The distance is available in the `hmDist` log variable |

Similarly all logs are in the `tdoa3` log group.

| Log       | Functionality                                                                      |
|-----------|------------------------------------------------------------------------------------|
| `hmDist`  | Measured distance to the anchor selected by the `hmAnchLog`` parameter [m]         |
| `hmTx`    | Transmission rate of TWR packets in hybrid mode [packets/s]                        |
| `hmSeqOk` | Rate of received TWR packets with matching seq nr in hybrid mode [packets/s]       |
| `hmEst`   | Rate of TWR measurements that are sent to the estimator in hybrid mode [samples/s] |

## Example configurations

### Use TDoA and TWR for positioning

| Parameter     | Value | Comment                                                    |
|---------------|-------|------------------------------------------------------------|
| `hmId`        | X     | Unique ids for each Crazyflie, it is set to 254 by default |
| `hmTwr`       | 1     | to start transmission                                      |
| `hmTwrEstPos` | 1     | to use TWR in position estimation                          |

### Measure distance between two Crazyflies

Settings for Crazyflie 1

| Parameter     | Value | Comment                |
|---------------|-------|------------------------|
| `hmId`        | 254   | the anchor id to use   |
| `hmTwr`       | 1     | to start transmission  |
| `hmAnchLog`   | 253   | the id of the other CF |

Settings for Crazyflie 2

| Parameter     | Value | Comment                |
|---------------|-------|------------------------|
| `hmId`        | 253   | the anchor id to use   |
| `hmTwr`       | 1     | to start transmission  |
| `hmAnchLog`   | 254   | the id of the other CF |

The distance to the other Crazyflie is available in the `tdoa3.hmDist` log variable.

### Use the Crazyflie as a new anchor

Assuming the Crazyflie is landed in a position where the estimated position is OK

| Parameter     | Value                                                                  |
|---------------|------------------------------------------------------------------------|
| `hmId`        | Unique anchor id                                                       |
| `hmTwr`       | 1 to start transmission                                                |
| `hmTwrTXPos`  | 2 to sample the current estimated position and use it in transmissions |
