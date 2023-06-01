---
title: CRTP - Communication with the Crazyflie
page_id: crtp_index
---

The packet protocol used to communicate with the Crazyflie is called CRTP. It
provides data to the packet that allows to route it to the different Crazyflie
subsystems. In the context of this documentation, the term `CRTP` is used both
to describe the packet protocol and its format as well as the collection of
data packets that are used to communicate with the Crazyflie.

Unless otherwise noted, this protocol documentation covers `Crazyflie 2.x`, the
`Crazyflie Bolt` and the `Roadrunner`.

## Protocol version and stability guarantee

In order to allow for improvement and breaking change the protocol is versioned.
The version is available using the [getProtocolVersion](crtp_platform#get-protocol-version) packet.

The current version is defined in the constant `CRTP_PROTOCOL_VERSION`.

When removing functionality from the protocol, packet will be deprecated for at least one version before being removed.
Deprecated functionality prints a Warning in the [console](crtp_console). This rule allows for the Crazyflie firmware
to evolve and remove old functionalities when needed and for a client or lib to have some compatibility guarantee.
For example if a client support the protocol version 5, it is guarantee to work with version 6 but should likely
not accept to connect a Crazyflie on version 7 of the protocol.

## Communication stack

The Crayzyflie communication is implemented as a stack of independent layers:

    +-------------------+
    +     Subsystems    +   <- Log/Param/Commander/...
    +-------------------+
    +       CRTP        +   <- (port, channel, payload)
    +-------------------+
    +       Link        +   <- Radio link or USB link
    +-------------------+
    +  Physical medium  +   <- radio or USB
    +-------------------+

 - The **physical layer** is responsible for transmitting packet to and from the
   Crazyflie. Currently USB and Radio are actively supported.
 - The **link**  is responsible of implementing safe and ordered packet channels
   to and from the Crazyflie. The link abstract the physical medium and
   implement one transmit and one receive packet channels to and from the
   Crazyflie.
 - **CRTP** implements *port* and *channel* information used to route the packet
   to various subsystem. *port* are very similar to UDP port, they are assigned
   to subsystems. *channel* are intended to be used in subsystems to route
   packets to different functionalities.
 - **Subsystems** implements the Crazyflie functionalities that can be
   controlled over CRTP. There is one *port* assigned to each *subsystem*.


## Link implementations

There is currently two actively supported link implementation. They are
documented on their own thread:
 - The radio link implements CRTP link over nRF24 compatible radios
 - The USB link implements CRTP link over USB to the Crazylfie 2.x USB port

### Packet ordering and real-time support

CTRP stands for `Crazy  RealTime Protocol`. It was designed to allow packet
prioritization to help real-time control of the Crazyflie.

The Link guarantees strict packet ordering **within a port**. Packets for
different ports can be re-organized and sent out of order.

If implemented, the packet prioritization works by assigning higher priority to
lower port number.

This allows use-case like uploading a trajectory while still controlling the
Crazyflie in real-time to work seamlessly as long as the trajectory upload
uses a higher port number as the real-time setpoints.

## CRTP packer metadata

Each CRTP packets carries one *port* number, a *channel* number as well as a
Payload:
 - The `port` range between 0 and 15 (4 bits)
 - The `channel` ranges between 0 and 3 (2 bits)
 - The payload is a data buffer of up to 31 bytes

The couple `port`:`channel` can be written separated by a color in this documentation.

## Null packet and special packets

The port:channel 15:3 is a special case and is called a *null packet*. This
packet is ignored by the Crazyflie and should be ignored by any CRTP lib on the
ground.

The radio link requires on packet to be sent in order to receive a packet, in this context the null packet is used to poll downlink data when there is no data to
be sent.

The NULL packet has been extensively used by links to implement side-channel
packets that are communicated outside the CRTP data flows. For example this is
used to implement bootloader packets that should be interpreted by the
Crazyflie's nRF51 radio chip without being passed to the STM32 application
processor.

## Port allocation

CRTP ports are allocated to subsytem in the Crazyflie. The intention is to
provide a bidirectional communication between one subsystem in the Crazyflie and
its handling on the ground.

| **Port** |  **Target**                                  |          **Used for**|
| ---------| ---------------------------------------------| ----------------------------------------------------------------|
|  0       | [Console](crtp_console.md)                   | Read console text that is printed to the console on the Crazyflie using consoleprintf|
|  2       | [Parameters](crtp_parameters.md)             | Get/set parameters from the Crazyflie. Parameters are defined using a [macro in the Crazyflie source-code](/docs/userguides/logparam.md)|
|  3       | [Commander](crtp_commander.md)               | Sending low level (instantaneous) control set-points for the roll/pitch/yaw/thrust regulators|
|  4       | [Memory access](crtp_mem.md)                 | Memory access for physical memories and register-mapped functionalities |
|  5       | [Data logging](crtp_log.md)                  | Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a [macro in the Crazyflie source-code](/docs/userguides/logparam.md)
|  6       | [Localization](crtp_localization.md)         | Packets related to localization|
|  7       | [Generic Setpoint](crtp_generic_setpoint.md) | Generic instantaneous setpoints (ie. position control and more) |
|  13      | [Platform](crtp_platform.md)                 | Used for misc platform control, like debugging and power off |
|  14      | Client-side debugging                        | Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself.|
|  15      | [Link layer](crtp_link.md)                   | Low level link-related service. For example *echo* to ping the Crazyflie |

Connection procedure
--------------------

Generaly speaking CRTP is connection-less and most of the subsystem in the
Crazyflie will strive to be stateless. This is not true for all the subsystem or
links though:
 - The USB link needs to be enabled in the Crazyflie using a USB control packet
 - The Radio link maintains two packet counters to ensure that there are no packet
   loss and strict packet ordering. This is called Safelink and needs to be
   enabled early when connecting a Crazyflie
 - The Log subsystem maintain a state of all the currently enabled log blocks
   and will continue sending data even if the link is lost. Therefore, the log
   subsystem implement a `reset` commands to reset the internal state.

When implementing a Crazyflie client the connection procedure will usually look
like:
 - Initialize the link
 - Initialize all the supported subsystem (this will be a no-op for most of them)
