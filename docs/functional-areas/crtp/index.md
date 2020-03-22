---
title: CRTP - Communication with the Crazyflie
page_id: ctrp_index
---

For communicating with the Crazyflie we have implemented our own
high-level protocol named CRTP (Crazy RealTime Protocol). It\'s a fairly
simple protocol using a number of target ports where data can be sent
and received in either direction, but in most cases the communication is
driven from the host. The protocol can be implemented on a number of
mediums, currently we have USB and Crazyradio support.

CRTP
====

Physical carriers
-----------------

Currently CRTP is supported over Crazyradio and USB (currently only for
Crazyflie 2.0).

 | Carrier          | Supports|
 | -----------------| -------------------|
 | Crazyradio (PA)  | Crazyflie 1.0/2.0|
 | USB              | Crazyflie 2.0|

Header
------

Each packet has a 1 byte header and can carry up to 31 bytes data
payload. The header field has the following layout:

       7    6    5    4    3    2    1    0
    +----+----+----+----+----+----+----+----+
    |       Port        |  Link   |  Chan.  |
    +----+----+----+----+----+----+----+----+

Where:

-   **Port**: Used to identify the functionality or task that is
    associated with this message
-   **Link**: Reserved for future use
-   **Channel**: Used to identify the sub-task/functionality

Port allocation
---------------

Current port allocation:

| **Port** |  **Target**                                                       |          **Used for**|
| ---------| ------------------------------------------------------------------| ----------------------------------------------------------------|
|  0       | [Console](/functional-areas/crtp/ctrp_console/)                   | Read console text that is printed to the console on the Crazyflie using consoleprintf|
|  2       | [Parameters](/functional-areas/crtp/ctrp_parameters/)             | Get/set parameters from the Crazyflie. Parameters are defined using a [macro in the Crazyflie source-code](/userguides/logparam/)|
|  3       | [Commander](/functional-areas/crtp/ctrp_commander/)               | Sending control set-points for the roll/pitch/yaw/thrust regulators|
|  4       | [Memory access](/functional-areas/crtp/ctrp_mem/)                 | Accessing non-volatile memories like 1-wire and I2C (only supported for Crazyflie 2.0)|
|  5       | [Data logging](/functional-areas/crtp/ctrp_log/)                  | Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a [macro in the Crazyflie source-code](/userguides/logparam/)|
|  6       | [Localization](/functional-areas/crtp/ctrp_localization/)         | Packets related to localization|
|  7       | [Generic Setpoint](/functional-areas/crtp/ctrp_generic_setpoint/) | Allows to send setpoint and control modes|
|  13      | Platform                                                          | Used for misc platform control, like debugging and power off|
|  14      | Client-side debugging                                             | Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself.|
|  15      | Link layer                                                        | Used to control and query the communication link|

Connection procedure
--------------------

CRTP is designed to be state-less, so there\'s no handshaking procedure
that is needed. Any command can be sent at any time, but for some
logging/param/mem commands the TOC (table of contents) needs to be
downloaded in order for the host to be able to send the correct
information. The implementation of the Pyton API will download the
param/log/mem TOC at connect in order to be able to use all the
functionality.
