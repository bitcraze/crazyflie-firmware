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
| 2 | GET_ITEM_V2 | Get an item from the TOC (up to 65535 entries) |
| 3 | GET_INFO_V2 | Get information about the TOC and LOG subsystem |

### GET_INFO_V2 (command 3)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 3 |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 3 |
| 1–2 | LOG_LEN | Number of log variables in the TOC (uint16, little-endian) |
| 3–6 | LOG_CRC | CRC32 fingerprint of the TOC |
| 7 | LOG_MAX_BLOCKS | Maximum number of log blocks that can be created |
| 8 | LOG_MAX_OPS | Maximum number of log operations (variable slots across all blocks) |

### GET_ITEM_V2 (command 2)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 2 |
| 1–2 | ID | ID of the item to retrieve (uint16, little-endian) |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 2 |
| 1–2 | ID | ID of the item returned (uint16, little-endian) |
| 3 | Type | Variable type (see type table) |
| 4.. | Group | Null-terminated string |
| .. | Name | Null-terminated string |

If the requested ID is out of range, the response is 1 byte (command only).

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
| 2 | DELETE_BLOCK | Delete a log block |
| 4 | STOP_BLOCK | Disable log block transmission |
| 5 | RESET | Delete all log blocks and stop all logging |
| 6 | CREATE_BLOCK_V2 | Create a new log block |
| 7 | APPEND_BLOCK_V2 | Append variables to an existing log block |
| 8 | START_BLOCK_V2 | Enable log block transmission at a given period |

### CREATE_BLOCK_V2 (command 6)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | CREATE_BLOCK_V2 | 6 |
| 1 | Block ID | Block identifier (uint8) |
| 2.. | ops_setting_v2[] | Zero or more entries, 3 bytes each: logType (uint8) + variable ID (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | CREATE_BLOCK_V2 | 6 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### APPEND_BLOCK_V2 (command 7)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | APPEND_BLOCK_V2 | 7 |
| 1 | Block ID | Block identifier (uint8) |
| 2.. | ops_setting_v2[] | One or more entries, 3 bytes each: logType (uint8) + variable ID (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | APPEND_BLOCK_V2 | 7 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### DELETE_BLOCK (command 2)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | DELETE_BLOCK | 2 |
| 1 | Block ID | Block identifier (uint8) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | DELETE_BLOCK | 2 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### START_BLOCK_V2 (command 8)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK_V2 | 8 |
| 1 | Block ID | Block identifier (uint8) |
| 2–3 | period_in_ms | Period in milliseconds (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | START_BLOCK_V2 | 8 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### STOP_BLOCK (command 4)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | STOP_BLOCK | 4 |
| 1 | Block ID | Block identifier (uint8) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | STOP_BLOCK | 4 |
| 1 | Block ID | Block identifier (uint8) |
| 2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### RESET (command 5)

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | RESET | 5 |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | RESET | 5 |
| 1 | (unused) | — |
| 2 | result | Always 0 |

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
