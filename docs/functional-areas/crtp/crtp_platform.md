---
title: Platform services
page_id: crtp_platform
---

This port implements miscellaneous platform-related functionality for the Crazyflie.

## CRTP channels

| port | channel | Function |
|------|---------|----------|
| 13   | 0       | [Platform commands](#platform-commands) |
| 13   | 1       | [Version commands](#platform-commands)  |
| 13   | 2       | [App channel](#app-channel) |
| 13   | 3       | *Reserved for future use* |

## Platform commands

The first byte describes the command:

| value | Command |
|-------|---------|
| 0     | [Set continuous wave](#set-continuous-wave) |
| 1     | [Request arm/disarm the system](#armdisarm-system) |

### Set continuous wave

Command and answer:

| Byte | Description |
|------|-------------|
| 0    | command setContinuousWave (0) |
| 1    | Enable |

If enable is not 0, the Crazyflie radio will start transmitting a continuous sine wave at the currently setup
freqency. The same packet is sent back to confirm the value has been set.

This command should only be sent over USB (it disables the radio communication).
It is used in production to test the Crazyflie radio path and should not be used outside of a lab or
other very controlled environment. It will effectively jam local radio communication on the channel.

### Arm/disarm system

Arm or disarm the system if possible.

Command:

| Byte | Description                           |
|------|---------------------------------------|
| 0    | command request arm/disarm system (1) |
| 1    | 0 = disarm, non-zero = arm the system |

Answer:

| Byte | Description                                          |
|------|------------------------------------------------------|
| 0    | command request arm/disarm system (1)                |
| 1    | success: 1 if the requested arming state was set     |
| 2    | isArmed: 0 = system is disarmed, 1 = system is armed |

## Version commands

The first byte describes the command:

| value | Command |
|-------|---------|
| 0     | Get protocol version |
| 1     | Get firmware version |
| 2     | Get device type name |

### Get protocol version

Command:

| Byte | Description |
|------|-------------|
| 0    | getProtocolVersion (0) |

Answer:

| Byte | Description |
|------|-------------|
| 0    | getProtocolVersion (0) |
| 1    | Version |

Returns the CRTP protocol version. See the
[protocol versioning and stability guarantee](index.md#protocol-version-and-stability-guarantee) documentation for more
information.

### Get firmware version

Command:

| Byte | Description |
|------|-------------|
| 0    | getFirmwareVersion (1) |

Answer:

| Byte | Description |
|------|-------------|
| 0    | getFirmwareVersion (1) |
| 1..  | Version string |

Returns a string representation of the current firmware version. This returns the GIT tag of the source code where the
firmware was built. For a release the version string will look like "2022.01". For a build between releases the
number of commit since the release will be added, for example "2022.01 +42".

### Get device type name

Command:

| Byte | Description |
|------|-------------|
| 0    | getDeviceTypeName (2) |

Answer:

| Byte | Description |
|------|-------------|
| 0    | getDeviceTypeName (2) |
| 1..  | Device type name string |

Returns a string representation of the device type the firmware is running on. The currently existing device types are:

| Device Type | Device type name |
|-------------|------------------|
| RZ10        | Crazyflie Bolt    |
| CF20        | Crazyflie 2.0    |
| CF2.1       | Crazyflie 2.1    |
| RR10        | Roadrunner 1.0   |

## App channel

The app channel is intended to be used by user apps on the Crazyflie and on the ground to exchange data. Every packet
sent and received from the app channel (port:channel) (13:2) will be available through the
[app channel API](/docs/userguides/app_layer.md#app-channel-packet-based-communication-between-the-crazyflie-and-the-python-lib).
