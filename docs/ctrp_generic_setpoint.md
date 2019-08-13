---
title: Generic Setpoint CRTP Port
page_id: ctrp_generic_setpoint
---


This port allows to send setpoints to the platform. The philosophy is to
be able to define setpoint packet format for each different use-case. As
such this is a generic port that has one channel and one main packet
format:

|  Port  | Channel  | Name|
|  ------| ---------| --------------------------------------------------|
|  7     | 0        | [Generic setpoint](#generic-setpoint)|

Generic setpoint
----------------

Generic setpoint packet format:

|  Byte  | Value    | Note|
|  ------| ---------| ---------------------------|
|  0      |ID       | ID of the setpoint packet|
|  1..   | Payload  | Format defined per ID|

Defined IDs:

 | ID  | Type|
 | ----| -----------------------------------------------------------------------|
 | 0   | [stop](#stop)|
 | 1   | [Velocity World](#velocity-world)|
  |2   | [Z Distance](#z-distance)|
 | 3   | [CPPM Emulation](#cppm-emulation)|
 | 4   | [Altitude Hold](#altitude-hold)|
 | 5   | [Hover](#hover)|
 | 6   | [Full State](#full-state)|
 | 7   | [Position](#position)|

### Stop

This is a setpoint with no payload that stops the motors and disables
the control loops. Should be sent when the Crazyflie is landed.

### Velocity World

Velocity setpoint in the world coordinate together with a YAW rotation
speed. Useful for a teleop mode in a local positioning system.

Payload format:

``` {.c}
struct velocityPacket_s {
  float vx;        // m in the world frame of reference
  float vy;        // ...
  float vz;        // ...
  float yawrate;  // deg/s
} __attribute__((packed));
```

### Z Distance

Set the Crazyflie absolute height and roll/pitch angles. Used for
Z-ranger.

Payload format:

``` {.c}
struct zDistancePacket_s {
  float roll;            // deg
  float pitch;           // ...
  float yawrate;         // deg/s
  float zDistance;        // m in the world frame of reference
} __attribute__((packed));
```

### CPPM Emulation

CRTP packet containing an emulation of CPPM channels Channels have a
range of 1000-2000 with a midpoint of 1500 Supports the ordinary RPYT
channels plus up to MAX\_AUX\_RC\_CHANNELS auxiliary channels. Auxiliary
channels are optional and transmitters do not have to transmit all the
data unless a given channel is actually in use (numAuxChannels must be
set accordingly)

Current aux channel assignments:

-   AuxChannel0: set high to enable self-leveling, low to disable

Payload format:

``` {.c}
#define MAX_AUX_RC_CHANNELS 10

static float s_CppmEmuRollMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuPitchMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuRollMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuPitchMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuYawMaxRateDps = 400.0f; // Used regardless of flight mode

struct cppmEmuPacket_s {
  struct {
      uint8_t numAuxChannels : 4;   // Set to 0 through MAX_AUX_RC_CHANNELS
      uint8_t reserved : 4;
  } hdr;
  uint16_t channelRoll;
  uint16_t channelPitch;
  uint16_t channelYaw;
  uint16_t channelThrust;
  uint16_t channelAux[MAX_AUX_RC_CHANNELS];
} __attribute__((packed));
```

### Altitude Hold

Set the Crazyflie vertical velocity and roll/pitch angle.

Payload format:

``` {.c}
struct altHoldPacket_s {
  float roll;            // rad
  float pitch;           // ...
  float yawrate;         // deg/s
  float zVelocity;       // m/s in the world frame of reference
} __attribute__((packed));
```

### Hover

Set the Crazyflie absolute height and velocity in the body coordinate
system.

Payload format:

``` {.c}
struct hoverPacket_s {
  float vx;           // m/s in the body frame of reference
  float vy;           // ...
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));
```

### Full State

Set the full state.

Payload format:

``` {.c}
struct fullStatePacket_s {
  int16_t x;         // position - mm
  int16_t y;
  int16_t z;
  int16_t vx;        // velocity - mm / sec
  int16_t vy;
  int16_t vz;
  int16_t ax;        // acceleration - mm / sec^2
  int16_t ay;
  int16_t az;
  int32_t quat;      // compressed quaternion, see quatcompress.h
  int16_t rateRoll;  // angular velocity - milliradians / sec
  int16_t ratePitch; //  (NOTE: limits to about 5 full circles per sec.
  int16_t rateYaw;   //   may not be enough for extremely aggressive flight.)
} __attribute__((packed));
```

#### Position

Set the absolute postition and orientation.

``` {.c}
 struct positionPacket_s {
   float x;     // Position in m
   float y;
   float z;
   float yaw;   // Orientation in degree
 } __attribute__((packed));
```
