---
title: Link layer services
page_id: crtp_link
---

This ports handle various link-related low level services. It most notably host the *null packet* used by the radio
link to pull downlink packets.

## CRTP channels

| port | channel | Function |
|------|---------|----------|
| 15   | 0       | [Echo](../crtp_platform#platform-commands) |
| 15   | 1       | [Source](../crtp_platform#platform-commands)  |
| 15   | 2       | [Sink](../crtp_platform#app-channel) |
| 15   | 3       | [*Null packets*](#null-packet) |

## Echo

Packets sent to the Echo channel are send back unaltered.

## Source

When receiving any packet on the source channel, the Crazyflie sends back a 32 bytes packet on the source channel.

Since protocol version 1, this packet contains the string "Bitcraze Crazyflie" followed by zeros. Before version 1 the
content of the packet was not defined. This allows to detect firmware protocol version bellow 1 (the getProtocolVersion
packet was only implemented after version 1 of the protocol).

## Sink

Packet sent to the sink channel are dropped and ignored.

## Null packet

Null packets must be dropped. The data part of NULL packet is used for some out-of-band communication at the link
level or by the bootloader. The Crazyflie firmware and lib should ignore them.
