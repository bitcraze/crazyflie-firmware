---
title: CPX - Crazyflie Packet eXchange
page_id: cpx_index
---

The Crazyflie Packet eXchange protocol (CPX) enables communication between
MCUs on the Crazyflie as well as between MCUs and a host. The CPX protocol
can both use CRTP and TCP as transport, and between MCUs on the Crazyflie
various interfaces (like UART and SPI) are used. The problem CPX aims to
solve is routing packages across multiple MCUs on the Crazyflie, a problem
that didn't exist when CRTP was designed.

Unlike CRTP, CPX might not be low latency. Where CRTP handling is fast,
no-blocking and handles small packets, CPX packages can handle larger packets,
communication is mostly blocking and if a MCU/host is overloaded congestion
will be created.

## Main design principles

CPX is a protocol with a source and destination as well as a function. The
destination is used to route the packet though intermediaries and the source
to route it back again. The function is much like the CRTP ports, and is used
to route data within the target MCU/host.

The protocol is meant to be a base, where application specific protocols can be
implemented on top of it. There's a few examples of this, like the AI-deck WiFi
image streamer and the GAP8 bootloader.

CPX is a hybrid of a packet and stream protocol. Packets below 30 bytes are
delivered in one chunk, but larger packets can be split up and delivered as
a stream of chunks where a header flag identifies the last chunk of the packet.

Packets are delivered in order, as are the chunks they might have been split into.
That being said the packets and chunks might be mixed in with packets and chunks
from other sources and/or other functions. If any target along the way becomes
overloaded with data, it will stop accepting new data and create congestion up
steam. Packets will not be dropped, but might not be delivered in a timely fashion.

## Transports

Since CPX is meant to be sent between different MCUs/host it can be implemented
on different transports. As long as the basic principles still stand, the transport
doesn't matter.

Today there's a few transports implemented in different parts of the Crazyflie:

* WiFi/TCP: CPX packages can be sent over WiFi/TCP using for example the AI-deck
* SPI: This is used between the GAP8 and the ESP32 on the AI-deck
* UART: This is used between the STM32 on the Crazyflie and the ESP32 on the AI-deck

## Data structure and stack

### Communication stack

{% ditaa --alt "Communication stack" %}
+-------------------+
+     Functions     +   <- Custom protocol (ex WiFi streamer or GAP8 bootloader)
+-------------------+
+       CPX         +   <- CPX routing header level
+-------------------+
+     Transport     +   <- CRTP, WiFI or custom for UART/SPI
+-------------------+   
+  Physical medium  +   <- 2.4 GHz, USB, UART or SPI
+-------------------+
{% endditaa %}

### Packet structure

Each physical medium and/or transport can have it's own MTU and will also add data
around the CPX packet to be able to use the medium and/or transport.

{% ditaa --alt "Packet structure" %}
+---------------------+
+     Medium data     +
+---------------------+
+   Transport data    +
+---------------------+
+     CPX Header      +
+---------------------+
+                     +
+                     +
+        DATA         +
+                     +
+                     +
+---------------------+
+   Transport data    +
+---------------------+
+     Medium data     +
+---------------------+
{% endditaa %}

As the packet moves up/down the stacks on each target data is added/removed and
the user only interacts with the CPX part of the packets.

### CPX header

The header for a CPX packet has the following format:

{% ditaa --alt "CPX Header" %}
    7        6         5        4        3        2        1       0
+--------+--------+--------+--------+--------+--------+--------+--------+
|  RSV   |   LP   |          SOURCE          |        DESTINATION       |
+--------+--------+--------+--------+--------+--------+--------+--------+
|     VERSION     |               FUNCTION                              |
+--------+--------+--------+--------+--------+--------+--------+--------+
{% endditaa %}

|    Field    | Size (bits) | Range  | Comments |
| ----------- | ----------- | ------ | -------- |
| RSV         |           1 |  0-1   | Reserved for future use |
| LP          |           1 |  0-1   | If a packet is split up in chunks, this will be 1 for the last chunk and 0 for the rest |
| SOURCE      |           3 |  0-7   | ID of the target sending the packet |
| DESTINATION |           3 |  0-7   | ID of the target which should receive the packet |
| VERSION     |           2 |  0-3   | CPX version |
| FUNCTION    |           6 |  0-63  | ID of the function where the target should route the packet internally |

There's no discovery for targets, so these will have to be in sync for all the targets routing packages.
There's also no discovery for functions, but these only have to be in sync for source and destination, since
intermediaries do not look at this value.

Here's a list of the current targets supported:

|   Target    |     Value   | Comment |
| ----------- | ----------- | -------- |
| STM32       |           1 | STM32 on Crazyflie |
| ESP32       |           2 | ESP32 on AI-deck |
| HOST        |           3 | Host computer (via WiFi) |
| GAP8        |           4 | GAP8 on AI-deck |

There's also a pre-defined set of functions, but others can be implemented if source
and destination is in sync:

|   Function  |     Value   | Comment |
| ----------- | ----------- | -------- |
| SYSTEM      |           1 | Used for target system functionality |
| CONSOLE     |           2 | Used for console printouts |
| CRTP        |           3 | Used to tunnel CRTP |
| WIFI CTRL   |           4 | Used for controlling the WiFi setup and signaling |
| APP         |           5 | Intended to be used with user applications |
| TEST        |          14 | Used for test functionality (ECHO, SOURCE and SINK) |
| BOOTLOADER  |          15 | Used to communicate with bootloaders (which support bootloader over CPX) |

### Physical medium

CPX supports a number of physical medium as well as various MTUs for them.

### Transport format

CPX support a number of transports as well as various MTUs for them. Each transport
will add data to the packets to be able to transfer them via the link, this data
will not be the same for different types of links.

#### UART

The packets for the UART transport link (ESP32 <> STM32) has be following format.
The MTU for the UART is 100 bytes (this includes CPX Header and DATA).

{% ditaa --alt "UART packet structure" %}
+----------------------+
+  Start byte (0xFF)   +
+----------------------+
+   Length (1 byte)    +
+----------------------+
+ CPX Header (2 bytes) +
+----------------------+
+                      +
+                      +
+         DATA         +
+                      +
+                      +
+----------------------+
+          CRC         +
+----------------------+
{% endditaa %}

| Byte number | Size | Comments |
| ----------- | ---- | -------- |
|           0 |    1 | Start byte for synchronization |
|           1 |    1 | This is the length of CPX Header + DATA |
|           2 |    2 | CPX Header (see above) |
|           3 |    n | Data size |
|       3+n+1 |    1 | XOR CRC |

There's no pins for flow control on the UART, instead the a special message is used
to avoid one side overflowing the other. For each packet received, the receiver will
send back ```0xFF 0x00``` (i.e a packet of size 0) to signal that the packet has been
queued and the sender can send the next one.

#### SPI

The packets for the SPI transport link (ESP32 <> GAP8) has be following format.
The MTU for the SPI is 1022 bytes (this includes CPX Header and DATA).

{% ditaa --alt "SPI packet structure" %}
+----------------------+
+   Length (2 bytes)   +
+----------------------+
+ CPX Header (2 bytes) +
+----------------------+
+                      +
+                      +
+         DATA         +
+                      +
+                      +
+----------------------+
{% endditaa %}

| Byte number | Size | Comments |
| ----------- | ---- | -------- |
|           0 |    2 | This is the length of CPX Header + DATA |
|           1 |    2 | CPX Header (see above) |
|           2 |    n | Data size |

For the SPI link there's flow control and the GAP8 is the SPI master. The ESP32
will signal when it can do a transaction and the GAP8 will signal when it can
to do a transaction. When both are ready a transaction is done.

#### WiFi

The packets for the WiFi transport link (ESP32 <> HOST) has be following format.
The MTU for the link is 1022 bytes (this includes CPX Header and DATA), but the actual
transferred data per packet will depend on the underlying TCP data.

{% ditaa --alt "WiFi packet structure" %}
+----------------------+
+         TCP          +
+----------------------+
+   Length (2 bytes)   +
+----------------------+
+ CPX Header (2 bytes) +
+----------------------+
+                      +
+                      +
+         DATA         +
+                      +
+                      +
+----------------------+
+         TCP          +
+----------------------+
{% endditaa %}

| Byte number | Size | Comments |
| ----------- | ---- | -------- |
|           - |    - | TCP/IP headers |
|           0 |    2 | This is the length of CPX Header + DATA |
|           1 |    2 | CPX Header (see above) |
|           2 |    n | Data size |
|           - |    - | TCP/IP |

The actual transferred data per packet (i.e what you would get out when receiving on the
host) is not defined and depends on the underlying TCP/IP stack. So when reading the socket
it's important to keep reading until you have received the correct amount of data.

### Packet splitting

As mentioned above CPX supports splitting packets into smaller chunks. The reason for this
is that not all MTUs are the same size, so a large packet might be split into smaller chunks
to be sent over a transport that has a smaller MTU.

When packets are split, the LP (last packet, see above) bit is set to 0 for all the chunks
except the last one, where the previous value is preserved. This means means that there's
support for splitting already split chunks.

The destination will have to re-assemble the packet as it seems fit, but this will be a higher
level protocol issue.

## APIs

The CPX implementation is split across multiple targets, repositories and languages. That being
said we've tried out best to synchronize the APIs, but there will be differences to please
double check the implementation in the repository where you're working.
