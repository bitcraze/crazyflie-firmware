---
title: Logging
page_id: crtp_log
---

For more information on how to use this and how this is implemented have
a look [here](/docs/userguides/logparam.md).

State machines
==============

Downloading the Table Of Contents
---------------------------------

![crtp log](/docs/images/crtp_log.png)

Communication protocol
======================

The log port is separated in 3 channels:

 | **Port**  | **Channel**  | **Function**|
 | ----------| -------------| ------------------
|  5         | 0            | Table of content access: Used for reading out the TOC|
|  5         | 1            | Log control: Used for adding/removing/starting/pausing log blocks|
|  5         | 2            | Log data: Used to send log data from the Crazyflie to the client|

Table of content access
-----------------------

This channel is used to download the Table Of Contents that contains all
the variables that are available for logging and what types they are.

The first byte of each messages correspond a command. All communication
on this port are initated by the client and all answer from the copter
will contain the same command byte.

|  TOC command byte   |Command    | Operation|
|  ------------------ |----------- |-----------------------------|
|  0                  |GET\_ITEM  | Get an item from the TOC|
|  1                  |GET\_INFO  | Get information about the TOC and the LOG subsystem| implementation

### Get TOC item

The GET\_ITEM TOC command permits to retrieved the log variables name,
group and types from the copter. This command is intended to be
requested from all the ID from 0 to LOG\_LEN (see GET\_INFO).

    Request (PC to Copter):
            +--------------+----+
            | GET_ITEM (0) | ID |
            +--------------+----+
    Length         1         1

    Answer (Copter to PC):
            +--------------+----+
            | GET_ITEM (0) | ID |                                        If index out of range
            +--------------+----+------+------------+--------------+
            | GET_ITEM (0) | ID | Type |   Group    |     Name     |     If returning Item
            +--------------+----+------+------------+--------------+
    Length        1          1     1    < Null terminated strings >

 | Byte   |Request fields  | Content|
 | ------| ---------------- |------------------------|
 | 0     | GET\_ITEM       | At 0 for GET\_ITEM operation|
 | 1     | ID              | ID of the item to be retrieved. The variables are numbered from 0 to LOG\_LEN (see GET\_INFO command)|

|  Byte  | Answer fields  | Content|
|  ------| ---------------| -------------------------------------------------------|
|  0     | GET\_ITEM      | 0 for GET\_ITEM operation|
|  1     | ID             | ID of the item returned|
|  2     | Type           | Variable type of the element. See variable types list|
|  3..   | Group          | Null-terminated string containing variable group|
|  ..    | Name           | Null-terminated string containing the variable name|

Type, group and name are not sent if the required ID is higher than
TOC\_LEN-1.

### Get Info

The get info command is intended to be requested first when connecting
to the copter. This permits to know the number of variable, the
limitations of the log implementation and the fingerprint of the log
variables.

    Request (PC to Copter):
            +--------------+
            | GET_INFO (1) |
            +--------------+
    Length         1

    Answer (Copter to PC):
            +--------------+---------+---------+-----------------+-------------+
            | GET_INFO (1) | LOG_LEN | LOG_CRC |  LOG_MAX_PACKET | LOG_MAX_OPS |
            +--------------+---------+---------+-----------------+-------------+
    Length        1             1         4             1               1

 | Byte   |Request fields  | Content|
 | ------ |----------------| ------------------------------|
 | 0      |GET\_INFO       | At 1 for GET\_INFO operation|

|  Byte  | Answer fields     | Content|
|  ------| ------------------| ----------------------------------------------------------|
|  0     | GET\_INFO        |  1 for GET\_INFO operation|
|  1     | LOG\_LEN         |  Number of log items contained in the log table of content|
|  2     | LOG\_CRC         |  CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC|
|  6     | LOG\_MAX\_PACKET  | Maximum number of log packets that can be programmed in the copter|
 | 7     | LOG\_MAX\_OPS     | Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming|

Log control
-----------

The log control channel permits to setup, activate, deactivate and
remove log packets. Like the TOC access channel the first data byte
represents a command.

|  Control command byte  | Command        | Operation|
|  ----------------------| ---------------| ---------------------------------------|
|  0                     | CREATE\_BLOCK  | Create a new log block|
|  1                     | APPEND\_BLOCK  | Append variables to an existing block|
|  2                     | DELETE\_BLOCK  | Delete log block|
|  3                     | START\_BLOCK   | Enable log block transmission|
|  4                     | STOP\_BLOCK    | Disable log block transmission|
|  5                     | RESET          | Delete all log blocks|

### Create block

### Append variable to block

### Delete block

### Start block

### Stop block

Log data
--------

The log data channel is used by the copter to send the log blocks at the
programmed rate. The packet format is

    Answer (Copter to PC):
            +----------+------------+---------//----------+
            | BLOCK_ID | TIME_STAMP | LOG VARIABLE VALUES |
            +----------+------------+---------//----------+
    Length        1          3           0 to 28

 | Byte  | Answer fields        | Content|
 | ------| --------------------- --------------------------------|
|  0     | BLOCK\_ID             |ID of the block|
|  1      |ID                    |Timestamp in ms from the copter startup as a little-endian 3 bytes integer|
|  4..    |Log variable values  | Packed log values in little endian format|
