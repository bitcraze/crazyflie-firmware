---
title: Localization CRTP port
page_id: crtp_localizaton
---


This port groups various packets related to localization. It exposes two
channels:

 | Port  | Channel  | Name|
 | ------| ---------| ----------------------|
 | 6     | 0        | External Position|
 | 6     | 1        | Generic localization|

External Position
-----------------

This packet is used to send the Crazyflie position as acquired by an
external system. The main use it to send the position acquired by a
motion capture system to push it in the Extended Kalman Filter to allow
the Crazyflie to calculate an estimate and control its state.

The packet format is:

``` {.c}
struct CrtpExtPosition
{
  float x; // in m
  float y; // in m
  float z; // in m
} __attribute__((packed));
```

Generic Localization
--------------------

This channel intends to host packets useful for the localization
subsystem. It has been created to serve the Loco Positioning System
packets but can be used for more general things like GPS NMEA or binary
streams. The format of the packet is:

|  Byte  | Value    | Note|
|  ------| ---------| ---------------------------------------|
|  0     | ID       | ID of the packet|
 | 1..   | Payload  | Packet payload. Format defined per ID|

|  ID  | Packet |
|  ----| ------------------------------|
|  2  |  LPP Short packet tunnel|
|  3  |  Enable emergency stop|
|  4  |  Reset emergency stop timeout|

### LPP Short packet tunnel

Packet used to send LPP short packet to the loco positioning system. The
payload is sent to the sytem as an [LPP Short
Packet](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/lpp/).

### Emergency stop

When received, the stabilizer loop is set in emergency stop mode which
stops all the motors. The loop stays in emergency stop mode until the
situation is reset.

### Reset emergency stop timeout

At startup the emergency stop timeout is disabled.

The first time this packet is receive, the emergency stop timeout is
enabled with a timeout of 1 second.

This packet should then be sent, and received by the Crazyflie, at least
once every 1 second otherwise the stabilizer loop will be set in
emergency stop and all motors will stop.
