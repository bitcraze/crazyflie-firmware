---
title: Parameters
page_id: crtp_parameters
---

The parameters system makes all the gettable and settable parameters of
the copter accessible. The copter holds a table of parameters which can
be retrieved. In this table every parameter name is associated with an
ID and a group name. Three IDs are used to acces the TOC and the
parameters:

|  Port |  Channel  | Function |
|  ------| ---------| --------------------------------------------------------------|
|  2   |   0        | [TOC access](#toc-access)|
|  2   |   1        | [Parameter read](#parameter-read)|
|  2   |   2        | [Parameter write](#parameter-write)|
|  2   |   3        | [Misc. commands](#misc-commands)|

## TOC access

These messages permit to access the parameters table of content. The
first byte of the message is a message ID, three messages ID are
defined:

 | Message ID  | Meaning in upstream packets  | Meaning in downstream packets |
 | ------------| -----------------------------| -------------------------------------|
 |0            | Reset TOC pointer            | Last TOC element|
 | 1           | Get next TOC element         | TOC element (ID, type, group, name)|
 | 3           | Get TOC CRC32                | Number of parameters, TOC CRC32|

The upstream ID are commands and are sent alone. The downstream has the
following formats:

    Bytes     1       1          1    Null terminated strings
            +---+------------+------+----------+--------------+
            | 0 |            |      |          |              |
            +---+  Param ID  | Type |  Group   |     Name     |
            | 1 |            |      |          |              |
            +---+------------+------+---+------+--------------+
            | 3 | Num. Param |  CRC32   |
            +---+------------+----------+
    Bytes     1       1           4

The parameters are sequentially requested by the PC until the end. When
the last parameter is reached it has the ID 0 \'Last TOC element\'. The
reset command permits to reset the TOC pointers so that the next sent
TOC element will be the first one. The \"Get TOC CRC\" command also
returns the number of parameters.

The CRC32 is a hash of the copter TOC. This is aimed at implementing
caching of the TOC in the PC Utils to avoid fetching the full TOC each
time the copter is connected.

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

## Parameter read


 | Byte  | Request fields  | Content
|  ------| ----------------| ---------------------------------------
|  0     | ID              | ID of the parameter to read (see TOC)



|  Byte  | Answer fields   |Content|
|  ------| ---------------| -----------------------------------------------------------------|
|  0     | ID             | ID of the parameter |
|  1-..  | value          | Value of the parameter. Size and format is described in the TOC |



The read request is a simple packet on channel 1. Crazyflie answers with
the value.

## Parameter write

 | Byte    | Request fields   |Content|
 | --------| ---------------- |---------------------------------------------------------|
 | 0      |  ID               |ID of the parameter to write|
 | 1-\...  | value           | Value to write. Size and format is described in the TOC|



 | Byte     |Answer fields   |Content|
|  -------- |--------------- |-----------------------------------------------------------------|
|  0        |ID              |ID of the parameter|
|  1-\...   |value           |Value of the parameter. Size and format is described in the TOC|



The write request is a simple packet on channel 2. Crazyflie sends back
the parameter value as an acknowledgement.

## Misc. commands

The following misc commands are implemented:

|  Code  | Command                                               |
|  ------| ------------------------------------------------------|
|  0x00  | [Set by name](#set-by-name)                           |
|  0x01  | [Value updated](#value-updated)                       |
|  0x02  | [Get extended type](#get-extended-type)               |
|  0x03  | [Persistent store](#persistent-store)                 |
|  0x04  | [Persistent get state](#persistent-get-state)         |
|  0x05  | [Persistent clear](#persistent-clear)                 |

### Set by name

|  Byte             | Request fields   | Content|
|  -----------------| ----------------| ---------------------------------------------|
|  0                | SET\_BY\_NAME   | 0x00 |
|  1-n              | group           | Name of the group |
|  n-(n+1)          | NULL            | 0 |
|  (n+1)-(n+m+1)    | name            | Name of the parameter |
|  (n+m+1)-(n+m+2)  | NULL            | 0 |
|  (n+m+2)-(n+m+3)  | TYPE            | Parameter type |
|  (n+m+3)-\...     | value           | Value. Size and format is described by type |



| Byte           | Answer fields  | Content|
| ---------------| ---------------| ---------------------------------------------------|
|  0             |  SET\_BY\_NAME  | 0x00|
|  1-n           |  group          | Name of the group|
|  n-(n+1)       |  NULL           | 0|
|  (n+1)-(n+m+1) |  name           | Name of the parameter|
|  (n+m+2)       |  NULL           | 0|
|  (n+m+3)       |  ERROR          | 0 if the parameter has been successfully written. Other code are taken from [errno C codes](http://www.virtsync.com/c-error-codes-include-errno). |


*Group* and *name* are ascii strings of size respectively *n* and *m*.
The type corresponds to the TOC type of the parameter. It is checked for
consistency.

This command is useful to set a parameter without having to fetch the
full TOC. It is enough to know the group, name and type of the parameter
to write it.

### Value updated

There is no request packet for this message, it is only sent by the Crazyflie.

| Byte           | Answer fields   | Content|
| ---------------| ----------------| ---------------------------------------------------|
|  0             |  VALUE\_UPDATED | 0x01                                               |
|  1-2           |  ID             | ID of the parameter                                |
|  3-\...        |value            |Value of the parameter. Size and format is described in the TOC|

This packet is send by the Crazyflie when a parameters has been modified in the firmware.
This can for example happen when an app is controlling the Crazyflie autonomously.

### Get extended type

Get the extended type of a parameter.

| Byte       | Request fields   | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_GET_EXTENDED_TYPE | 0x02                             |
| 1-2        | ID               | ID of the parameter              |

| Byte       | Answer fields    | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_GET_EXTENDED_TYPE | 0x02                             |
| 1-2        | ID               | ID of the parameter              |
| 3          | Extended type    | A bit field of extended types, see below |

The extended type describes extended properties of the parameter. Currently only one extended type is available.

| Value      | Exended type     | Description |
|------------|------------------|-------------|
| 0x01       | PERSISTENT       | The parameter can be stored in persistent memory |

### Persistent store

Store the current value of a parameter to persistent storage. The parameter will be set to this value.
after reboot.

| Byte       | Request fields   | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_STORE | 0x03                             |
| 1-2        | ID               | ID of the parameter              |

| Byte       | Answer fields    | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_STORE | 0x03                             |
| 1-2        | ID               | ID of the parameter              |
| 3          | result           | 0x00 == success<br>0x02 (ENOENT) == parameter ID does not exist |

### Persistent get state

Get the persistence state of a parameter.

| Byte        | Request fields       | Content                           |
| ------------| ---------------------| ----------------------------------|
| 0           | PERSISTENT_GET_STATE | 0x04                              |
| 1-2         | ID                   | ID of the parameter               |

| Byte        | Answer fields        | Content                           |
| ------------| ---------------------| ----------------------------------|
| 0           | PERSISTENT_GET_STATE | 0x04                              |
| 1-2         | ID                   | ID of the parameter               |
| 3           | result               | 0x00 == parameter is not stored (use default value)<br>0x01 == parameter is stored<br>0x02 (ENOENT) == parameter ID does not exist  |
| 4 + ts      | default value        | The default value that is used if no value is stored in persistent memory |
| 4 + ts + ts | [stored value]       | The stored value, if it is stored |

ts == the type size, described in the TOC

If the result is an error (0x02) the packet is terminated after 4 bytes.


### Persistent clear

Clear the persistent data for a parameter. After reboot the parameter will be set to the default value.

| Byte       | Request fields   | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_CLEAR | 0x05                             |
| 1-2        | ID               | ID of the parameter              |

| Byte       | Answer fields    | Content                          |
| -----------| -----------------| ---------------------------------|
| 0          | PERSISTENT_CLEAR | 0x05                             |
| 1-2        | ID               | ID of the parameter              |
| 3          | result           | 0x00 == success<br>0x02 (ENOENT) == parameter ID does not exist or other error |
