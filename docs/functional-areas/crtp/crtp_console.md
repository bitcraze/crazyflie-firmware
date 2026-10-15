---
title: Console
page_id: crtp_console
---

CRTP port 0 carries console output from the Crazyflie and registered sources
such as the Camera Deck. Clients can discover and enable registered sources,
then display their output separately from the local console. Console output
is intended as UTF-8 but travels as binary bytes. Code points may span
packets, and invalid UTF-8 may occur. Clients should use an incremental,
loss-tolerant decoder and must not treat packet boundaries as line or record
boundaries.

Channels 1 through 3 require CRTP protocol version 13.

## Sources and catalog

A source is a named console stream. The Crazyflie's local console is always
available on channel 0. Other firmware components register sources such as
`deck:bcCam`. A source path is non-empty UTF-8 with non-empty colon-separated
segments. The source catalog maps each path to an ID and an enabled state.
IDs start at 0 in registration order, and each source starts disabled. The
catalog holds at most 255 sources, with IDs 0x00 through 0xfe. ID 0xff selects
all registered sources in a control request.

Firmware code registers a `ConsoleSource` with `consoleSourceRegister()` before
startup completes. The `ConsoleSource` and its unchanged path must remain valid
for the firmware lifetime. Once registered, the node pointer is the handle for
`consoleSourceIsEnabled()` and `consoleSourceSend()`. Console closes
registration with `consoleSourceFreeze()`. The catalog then stays fixed until
reboot, even if a source becomes temporarily unavailable.

Clients use channel 3 to map IDs to paths, channel 2 to enable sources, and
channel 1 to receive their output. A valid catalog or control request before
the catalog is frozen returns `EAGAIN`; clients should retry after startup.

## Channel 0: local console

Channel 0 carries console output from the Crazyflie's STM32. Its payload
contains 0 to 30 bytes. Its implicit source is `cf:stm32`, which is not listed
in the source catalog.

## Channel 1: sourced console

```text
Port 0, channel 1, Crazyflie -> client
+-----------+--------------------+
| source ID | console bytes      |
+-----------+--------------------+
    u8            0..29
```

## Channels 2 and 3: commands and responses

Every response starts with the request command. The
second byte is a status: zero on success or a
[firmware error number](crtp_error_numbers.md). Error responses contain only
those two bytes. Empty requests receive no response.

### Channel 2: runtime control

`SET_ENABLED` is idempotent and lasts until changed or the Crazyflie reboots.

```text
Request:  [0x00, source_id, enabled]
Success:  [0x00, 0x00, source_id, enabled]
Error:    [0x00, errno]
```

`enabled` is 0 or 1.

An unknown command returns `[command, ENOSYS]`, an invalid payload returns
`[command, EINVAL]`, and an unknown source ID returns `[0x00, ENOENT]`.

A successful disable response marks a boundary in the transmit queue. Any
sourced packet accepted before the disable is queued ahead of the response.
No further packets from that source are queued until it is enabled again.

### Channel 3: source catalog (TOC)

The source catalog uses the following commands:

```text
GET_ITEM request:  [0x00, source_id]
GET_ITEM success:  [0x00, 0x00, source_id, source_path ...]

GET_INFO request:  [0x01]
GET_INFO success:  [0x01, 0x00, source_count, catalog_crc_u32_le]

Error response:    [command, errno]
```

The source path occupies the remainder of a successful `GET_ITEM`; it is not NUL
terminated and is at most 27 encoded bytes in the 30-byte CRTP payload. Paths
are for display and filtering; for example, the Camera Deck source is
`deck:bcCam`. The protocol does not identify individual deck instances.

The checksum is CRC-32/ISO-HDLC over each entry in ID order as one ID byte
followed by its path bytes. Its reflected polynomial is `0xEDB88320`, initial
value `0xffffffff`, and final XOR `0xffffffff`; the response encodes the
resulting integer little-endian.

An unknown item returns `ENOENT`, an unknown command returns `ENOSYS`, and an
invalid payload returns `EINVAL`.

The Camera Deck's UART binding and buffering behavior are described in
[Camera Deck console](../camera-deck-console.md).
