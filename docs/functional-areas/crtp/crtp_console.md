---
title: Console
page_id: crtp_console
---

CRTP port 0 carries local and dynamically bound console byte streams. Console
content is canonically UTF-8 but transported as binary: code points may span
packets and invalid UTF-8 may occur. Clients should use an incremental,
loss-tolerant decoder and must not infer line or record boundaries from CRTP
packets.

The sourced Console extension on channels 1 through 3 is available from CRTP
protocol version 13. Channel 0 retains its legacy format.

## Channel 0: local console

Channel 0 is the existing Crazyflie-local stream. Its payload is 0 to 30 bytes
and has the implicit source `cf:stm32`. It is unchanged and is not listed in
the source TOC.

## Channel 1: sourced console

Sourced console output is disabled after every Crazyflie boot.

```text
Port 0, channel 1, Crazyflie -> client
+-----------+--------------------+
| source ID | console bytes      |
+-----------+--------------------+
    u8            0..29
```

IDs 0x00 through 0xfe identify entries in the boot-lifetime source catalog.
Packet boundaries are adapter boundaries only. A Common Link frame larger than
29 bytes is split across multiple channel-1 packets with the same source ID.

## Channel 2: runtime control

`SET_ENABLED` is idempotent and lasts until changed or the Crazyflie reboots.
Source ID 0xff means all catalog entries.

```text
Request:  [0x00, source_id, enabled]
Response: [0x00, source_id, enabled, result]
```

`enabled` is 0 or 1. `result` is zero on success or an errno-compatible status.
Before the source catalog is frozen, a valid request returns `EAGAIN` in the
normal response's `result` field. Clients must be prepared to retry it after
the Crazyflie finishes starting.

An empty request is not answered. A request with an unknown command byte gets
the two-byte response `[command, ENOSYS]`; a known command with an invalid
payload gets `[command, EINVAL]`. This short error form lets clients distinguish
unsupported future commands from malformed versions of known commands.

Disabling a source does not discard an already accepted upstream frame; it
withholds upstream credit until the source is enabled and downstream CRTP
capacity is available. A successful single-source or all-source disable
response is a transmit-ordering barrier: a sourced packet that was accepted
before disable is queued ahead of the response, and no sourced packet for the
disabled source is queued after it until that source is enabled again.

## Channel 3: source TOC

The TOC uses the original 8-bit log/parameter command shape, without V1/V2 in
the command names.

```text
GET_ITEM request:  [0x00, source_id]
GET_ITEM response: [0x00, source_id, source_path ...]

GET_INFO request:  [0x01]
GET_INFO response: [0x01, source_count, catalog_crc_u32_le]

Error response:    [command, errno]
```

The source path occupies the remainder of `GET_ITEM`; it is not NUL terminated.
Paths are non-empty UTF-8 with non-empty colon-separated segments and are
unique in the catalog, for example `deck:bcCam` or `cf:nRF51`. They are for
display and filtering, not stable device identity.

The catalog is frozen for the Crazyflie boot. IDs are contiguous from zero and
remain assigned if a source becomes temporarily unavailable. The CRC is the
firmware CRC-32 over each entry in ID order as `[id byte][source path bytes]`.
Before freeze, a valid `GET_INFO` or `GET_ITEM` request returns `EAGAIN`; clients
must retry after startup. An unknown item returns `ENOENT`. As on channel 2, an
empty request is not answered, an unknown command returns `ENOSYS`, and a known
command with an invalid payload returns `EINVAL`.

## Camera Deck binding

The static `bcCam` UART service currently uses the generic Common Link
endpoint to enumerate the immutable target service catalog and bind a
compatible `bitcraze.console` service to source `deck:bcCam`. It does not
assume a service handle or descriptor ordinal. A future deck/service discovery
layer can move this composition out of the driver without changing either
console protocol.

Crazyflie grants the Console service one Common Link receive slot only while
the source is enabled and no previous Console frame remains to be queued to
CRTP. The slot is independent of the Control service receive slot, so a stalled
Control probe does not prevent Console traffic. While this source is enabled
and its Console service is bound, the bcCam startup-recovery watchdog is
suspended so that it does not reset the deck during diagnosis. Disabling the
source starts a fresh recovery timeout if Control is still stalled.
Consequently CRTP congestion propagates to the Camera Deck spool, which is the
intentional best-effort loss boundary.
