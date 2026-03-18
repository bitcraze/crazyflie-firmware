---
title: Logging
page_id: crtp_log
---

For more information on how to use this and how this is implemented have
a look [here](/docs/userguides/logparam.md).

The log port is separated in these channels:

 | **Port**  | **Channel**  | **Function**|
 | ----------| -------------| ------------------
|  5         | 0            | [Table of Contents access](#table-of-contents-access): Used for reading out the TOC|
|  5         | 1            | [Log control](#log-control): Used for adding/removing/starting/pausing log blocks|
|  5         | 2            | [Log data](#log-data): Used to send log data from the Crazyflie to the client|

From here on in this section, only the payload of each message is described.

## State machines

### Downloading the Table of Contents

![crtp log](/docs/images/crtp_log.png)

## Table of Contents access

- Port: 5
- Channel: 0

This channel is used to download the Table of Contents (TOC) that contains all
the variables that are available for logging and what types they are.

The first byte of each message's payload corresponds to a command. All communication
is initiated by the client and all answers from the copter
will contain the same command byte.

| TOC command byte | Command | Operation |
|---|---|---|
| 0x02 | [GET_ITEM_V2](#get_item_v2-command-0x02) | Get an item from the TOC (up to 65535 entries) |
| 0x03 | [GET_INFO_V2](#get_info_v2-command-0x03) | Get information about the TOC and LOG subsystem |

### GET_ITEM_V2 (command 0x02)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 0x02 |
| 1–2 | ID | ID of the item to retrieve (uint16, little-endian) |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 0x02 |
| 1–2 | ID | ID of the item returned (uint16, little-endian) |
| 3 | Type | Variable type (see type table) |
| 4.. | Group | Null-terminated string |
| .. | Name | Null-terminated string |

If the requested ID is out of range, the response is 1 byte (command only).

### GET_INFO_V2 (command 0x03)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 0x03 |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 0x03 |
| 1–2 | LOG_LEN | Number of log variables in the TOC (uint16, little-endian) |
| 3–6 | LOG_CRC | CRC32 fingerprint of the TOC |
| 7 | LOG_MAX_BLOCKS | Maximum number of log blocks that can be created |
| 8 | LOG_MAX_OPS | Maximum number of log operations (variable slots across all blocks) |

## Log control

- Port: 5
- Channel: 1

The log control channel permits to setup, activate, deactivate and
remove log blocks.

The first byte of each message's payload corresponds to a command. All communication
is initiated by the client and all answers from the copter
will contain the same command byte.

| Control command byte | Command | Operation |
|---|---|---|
| 0x02 | [DELETE_BLOCK](#delete_block-command-0x02) | Delete a log block |
| 0x03 | [START_BLOCK](#start_block-command-0x03) | Enable log block transmission at a given period |
| 0x04 | [STOP_BLOCK](#stop_block-command-0x04) | Disable log block transmission |
| 0x05 | [RESET](#reset-command-0x05) | Delete all log blocks and stop all logging |
| 0x06 | [CREATE_BLOCK_V2](#create_block_v2-command-0x06) | Create a new log block |
| 0x07 | [APPEND_BLOCK_V2](#append_block_v2-command-0x07) | Append variables to an existing log block |
| 0x08 | [START_BLOCK_V2](#start_block_v2-command-0x08) | Enable log block transmission at a given period |

### DELETE_BLOCK (command 0x02)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | DELETE_BLOCK | 0x02 |
| 1 | Block ID | Block identifier (uint8) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | DELETE_BLOCK | 0x02 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### START_BLOCK (command 0x03)

> **Deprecated.** Use [START_BLOCK_V2](#start_block_v2-command-0x08) for new implementations. This command remains supported for backwards compatibility.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK | 0x03 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | Period | Transmission period in units of 10 ms (uint8, max 2550 ms) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK | 0x03 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### STOP_BLOCK (command 0x04)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | STOP_BLOCK | 0x04 |
| 1 | Block ID | Block identifier (uint8) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | STOP_BLOCK | 0x04 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### RESET (command 0x05)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | RESET | 0x05 |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | RESET | 0x05 |
| 1 | (unused) | — |
| 2 | result | Always 0 |

### CREATE_BLOCK_V2 (command 0x06)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | CREATE_BLOCK_V2 | 0x06 |
| 1 | Block ID | Block identifier (uint8) |
| 2.. | ops_setting_v2[] | Zero or more entries, 3 bytes each: logType (uint8) + variable ID (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | CREATE_BLOCK_V2 | 0x06 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### APPEND_BLOCK_V2 (command 0x07)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | APPEND_BLOCK_V2 | 0x07 |
| 1 | Block ID | Block identifier (uint8) |
| 2.. | ops_setting_v2[] | One or more entries, 3 bytes each: logType (uint8) + variable ID (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | APPEND_BLOCK_V2 | 0x07 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### START_BLOCK_V2 (command 0x08)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK_V2 | 0x08 |
| 1 | Block ID | Block identifier (uint8) |
| 2–3 | period_in_ms | Period in milliseconds (uint16, little-endian, max 65535 ms) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK_V2 | 0x08 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

## Log data

- Port: 5
- Channel: 2

The log data channel is used by the copter to send the log blocks at the
programmed rate.

| Byte | Field | Content |
|------|-------|---------|
| 0 | BLOCK_ID | ID of the log block |
| 1–3 | TIMESTAMP | Timestamp in ms since copter startup (uint24, little-endian) |
| 4.. | values | Packed log variable values (0 to 26 bytes, little-endian) |
