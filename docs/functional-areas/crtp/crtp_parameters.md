---
title: Parameters
page_id: crtp_parameters
---

The parameters system makes all the gettable and settable parameters of
the copter accessible. The copter holds a table of parameters which can
be retrieved. In this table every parameter name is associated with an
ID and a group name. Three IDs are used to acces the Table of Contents (TOC) and the
parameters:

|  Port |  Channel  | Function |
|  ------| ---------| --------------------------------------------------------------|
|  2   |   0        | [Table of Contents access](#toc-access)|
|  2   |   1        | [Parameter read](#parameter-read)|
|  2   |   2        | [Parameter write](#parameter-write)|
|  2   |   3        | [Miscellaneous commands](#miscellaneous-commands)|

From here on in this section, only the payload of each message is described.

## Table of Contents access

- Port: 2
- Channel: 0

These messages permit to access the parameters Table of Contents. The
first byte of each message's payload is a command byte. All communication is
initiated by the client and answers from the Crazyflie contain the same
command byte.

| TOC command byte | Command | Operation |
|---|---|---|
| 2 | GET_ITEM_V2 | Get a parameter entry from the TOC by ID |
| 3 | GET_INFO_V2 | Get the number of parameters and TOC CRC32 |

### GET_ITEM_V2 (command 2)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 2 |
| 1–2 | ID | ID of the parameter to retrieve (uint16, little-endian) |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_ITEM_V2 | 2 |
| 1–2 | ID | ID of the parameter returned (uint16, little-endian) |
| 3 | type | Parameter type (see [type table](#type-table) below) |
| 4.. | group | Null-terminated string |
| .. | name | Null-terminated string |

If the requested ID is out of range, the response is 1 byte (command only).

#### Type table

The type is one byte describing the parameter type:

|  Type code |  C type     | Python unpack |
|  -----------| -----------| ---------------|
| 0x08      |  uint8\_t   |  \'&lt;B \'|
|  0x09      |  uint16\_t |  \'&lt;H\' |
|  0x0A      |  uint32\_t |  \'&lt;L\' |
|  0x0B      |  uint64\_t |  \'&lt;Q\' |
|  0x00      |  int8\_t   |  \'&lt;b\' |
|  0x01      |  int16\_t  |  \'&lt;h\' |
|  0x02      |  int32\_t  |  \'&lt;i\' |
|  0x03      |  int64\_t  |  \'&lt;q\' |
|  0x05      |  FP16      |  \'\'    |
|  0x06      |  float     |  \'&lt;f\' |
|  0x07      |  double    |  \'&lt;d\' |

### GET_INFO_V2 (command 3)

Request (PC to Crazyflie):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 3 |

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_INFO_V2 | 3 |
| 1–2 | paramsCount | Number of parameters in the TOC (uint16, little-endian) |
| 3–6 | paramsCRC | CRC32 fingerprint of the TOC |

`paramsCRC` is a hash of the TOC contents and can be used to cache the TOC on the client side, avoiding a full re-fetch on every connection.

## Parameter read

- Port: 2
- Channel: 1

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | ID | ID of the parameter to read (uint8) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | ID | ID of the parameter |
| 1.. | value | Value of the parameter (size and format described in TOC) |

The read request is a simple packet on channel 1. Crazyflie answers with
the value.

## Parameter write

- Port: 2
- Channel: 2

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | ID | ID of the parameter to write (uint8) |
| 1.. | value | Value to write (size and format described in TOC) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | ID | ID of the parameter |
| 1.. | value | Value of the parameter (size and format described in TOC) |

Crazyflie sends back the parameter value as an acknowledgement.

## Miscellaneous commands

- Port: 2
- Channel: 3

The following miscellaneous commands are implemented:

|  Code  | Command                                               |
|  ------| ------------------------------------------------------|
|  0x00  | [Set by name](#set-by-name)                           |
|  0x01  | [Value updated](#value-updated)                       |
|  0x02  | [Get extended type](#get-extended-type) (deprecated, use 0x07) |
|  0x03  | [Persistent store](#persistent-store)                 |
|  0x04  | [Persistent get state](#persistent-get-state)         |
|  0x05  | [Persistent clear](#persistent-clear)                 |
|  0x06  | [Get default value](#get-default-value) (deprecated, use 0x08) |
|  0x07  | [Get extended type V2](#get-extended-type-v2) |
|  0x08  | [Get default value V2](#get-default-value-v2) |

### Set by name

*Group* and *name* are ASCII strings of length *n* and *m* respectively.
The type corresponds to the TOC type of the parameter and is checked for consistency.

This command is useful to set a parameter without having to fetch the
full TOC. It is enough to know the group, name and type of the parameter
to write it.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | SET_BY_NAME | 0x00 |
| 1..n | group | Group name (null-terminated string) |
| n+1..n+m+1 | name | Parameter name (null-terminated string) |
| n+m+2 | TYPE | Parameter type |
| n+m+3.. | value | Value (size and format described by type) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | SET_BY_NAME | 0x00 |
| 1..n | group | Group name (null-terminated string) |
| n+1..n+m+1 | name | Parameter name (null-terminated string) |
| n+m+2 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### Value updated

There is no request packet for this message, it is only sent by the Crazyflie.

Answer (Crazyflie to PC):

| Byte | Field | Content |
|------|-------|---------|
| 0 | VALUE_UPDATED | 0x01 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3.. | value | Value of the parameter (size and format described in TOC) |

This packet is sent by the Crazyflie when a parameter has been modified in the firmware.
This can for example happen when an app is controlling the Crazyflie autonomously.

### Get extended type
**Deprecated**: Use [Get extended type V2 (0x07)](#get-extended-type-v2) instead. This command may have ambiguous responses if the extended_type value equals an error code (e.g., PARAM_NOT_FOUND=2). Currently not an issue (only extended_type=1 exists), but the V2 command provides an unambiguous format.

Get the extended type of a parameter.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_EXTENDED_TYPE | 0x02 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_EXTENDED_TYPE | 0x02 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | extended type | Bit field of extended types (see below) |

The extended type describes extended properties of the parameter. Currently only one extended type is available.

| Value      | Extended type    | Description |
|------------|------------------|-------------|
| 0x01       | PERSISTENT       | The parameter can be stored in persistent memory |

### Persistent store

Store the current value of a parameter to persistent storage. The parameter will be set to this value
after reboot.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_STORE | 0x03 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_STORE | 0x03 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### Persistent get state

Get the persistence state of a parameter.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_GET_STATE | 0x04 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_GET_STATE | 0x04 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result | 0x00 = not stored, 0x01 = stored, [error number](crtp_error_numbers.md) on failure (packet ends here) |
| 4..4+ts–1 | default value | Default value (ts = type size from TOC) |
| 4+ts..4+2ts–1 | stored value | Stored value (only present if result == 0x01) |

### Persistent clear

Clear the persistent data for a parameter. After reboot the parameter will be set to the default value.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_CLEAR | 0x05 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | PERSISTENT_CLEAR | 0x05 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result | 0 on success, [error number](crtp_error_numbers.md) on failure |

### Get default value

**Deprecated**: Use [Get default value V2 (0x08)](#get-default-value-v2) instead. This command has ambiguous responses for U8 parameters with default value 2 (PARAM_NOT_FOUND).

Get the default value of a parameter. The default value is the value the parameter has at startup if not overridden by persistent storage.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_DEFAULT_VALUE | 0x06 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_DEFAULT_VALUE | 0x06 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result/value | On error: [error number](crtp_error_numbers.md); on success: first byte of default value |
| 4.. | value continued | Remaining bytes of default value (if applicable) |

**Protocol ambiguity**: For U8 parameters with default value 2, the success response `[0x06, ID_L, ID_H, 0x02]` is identical to the error response with PARAM_NOT_FOUND. Clients should check the packet size to distinguish, or preferably use the V2 command.

### Get extended type V2

This command provides unambiguous responses compared to the deprecated [Get extended type (0x02)](#get-extended-type).

Get the extended type of a parameter with an explicit status byte.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_EXTENDED_TYPE_V2 | 0x07 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_EXTENDED_TYPE_V2 | 0x07 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result | 0 on success, [error number](crtp_error_numbers.md) on failure (packet ends here) |
| 4 | extended type | Bit field of extended types (only present on success) |

### Get default value V2

This command provides unambiguous responses compared to the deprecated [Get default value (0x06)](#get-default-value).

Get the default value of a parameter with an explicit status byte.

Request:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_DEFAULT_VALUE_V2 | 0x08 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |

Answer:

| Byte | Field | Content |
|------|-------|---------|
| 0 | GET_DEFAULT_VALUE_V2 | 0x08 |
| 1–2 | ID | ID of the parameter (uint16, little-endian) |
| 3 | result | 0 on success, [error number](crtp_error_numbers.md) on failure (packet ends here) |
| 4.. | default value | Default value (only present on success, size described in TOC) |
