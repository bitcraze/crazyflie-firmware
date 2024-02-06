---
title: Memory access
page_id: crtp_mem
---


Using the memory access gives the possibility to:

-   Get information about which memories are available
-   Read/write/erase memories

Currently the following memories are supported:

-   Crazyflie 2.X onboard EEPROM
-   Crazyflie 2.X expansion board 1-wire memories

There\'s more information available for how the EEPROM is structured and
how the 1-wire memories work and are structured.

## Logical flow

Getting information and reading/writing the memories is optional for the
clients, but the -Crazyflie Python Client- always downloads information
about the memories on connect.

![crtp mem](/docs/images/crtp_mem.png)

## Communication protocol

The memory port uses 3 different channels:

  **Port**   **Channel**   **Function**
  ---------- ------------- ---------------------------------------------------------------------
  4          0             Get information about amount and types of memory as well as erasing
  4          1             Read memories
  4          2             Write memories

### Channel 0: Info/settings

This channel is used to get the number of memories present, information
about the memories and the possibility to mass erase memories. The first
byte of every packet is a command byte:

|  Command byte   |Command              |Operation|
|  -------------- |-------------------- |--------------------------------|
|  1             | GET\_NBR\_OF\_MEMS  | Get the number of memories|
|  2             | GET\_MEM\_INFO      | Get information about a memory|
|  3             | SET\_MEM\_ERASE     | Mass erase a memory|

### GET\_NBR\_OF\_MEMS

This command is used to get the number of memories present.

The request from host to Crazyflie:

|  Byte   |Field                   |Value   |Length   |Comment|
|  ------ |-----------------------| ------- |-------- |------------------|
|  0      |GET\_NUMBER\_OF\_MEMS  | 0x01    |1        |The command byte|

Reply from Crazyflie to host:

|  Byte  | Field                  | Value  | Length  | Comment|
|  ------| -----------------------| -------| --------| ---------------------------|
|  0     | GET\_NUMBER\_OF\_MEMS  | 0x01   | 1       | The command byte|
|  1     | Number of memories     |        | 1       | The number of memories preset (all types)|

Example where there are 3 memories present on the Crazyflie:

    Host-to-Crazyflie: <port/chan> 0x01
    Crazyflie-to-Host: <port/chan> 0x01 0x03

### GET\_MEM\_INFO

This command is used to get information about a memory give it\'s id.
The id of memories is sequential and is from 0 to one less than the
returned number of memories from GET\_NUMBER\_OF\_MEMS.

The request from host to Crazyflie:

|  Byte  | Field           | Value  | Length  | Comment|
|  ------| ----------------| -------| --------| --------------------------------------------|
|  0    |  GET\_MEM\_INFO  | 0x02   | 1       | The command byte|
|  1     | MEM\_ID          |       | 1       | A memory id that is 0 \<= id \< NBR\_OF\_MEMS|

Reply from Crazyflie to host if the id is valid:

 | Byte  | Field           | Value  | Length  | Comment|
 | ------| ----------------| -------| --------| -------------------|
|  0     | GET\_MEM\_INFO  | 0x02   | 1       | The command byte|
|  1     | MEM\_ID         |        | 1       | The memory id|
|  2     | MEM\_TYPE       |        | 1       | The memory type (see below)|
|  3     | MEM\_SIZE       |        | 4       | The size in bytes of the memory|
|  7     | MEM\_ADDR       |        | 8       | The address of the memory (only valid for 1-wire memories)|

Where the MEM\_TYPE field is:

|  MEM\_TYPE  | Memory type   |Comment |
|  -----------| ------------- |---------|
|  0          | I2C          | |
|  1          | 1-wire        ||

Reply from Crazyflie to host if the id is not valid:

 | Byte  | Field           | Value  | Length  | Comment
 | ------| ----------------| -------| --------| ------------------
 | 0     | GET\_MEM\_INFO  | 0x02   | 1       | The command byte
 | 1     | MEM\_ID         |        | 1       | The memory id

Example of requesting the information for a 1-wire memory with
MEM\_ID=1, MEM\_SIZE=112bytes, MEM\_ADDR=0x1234567890ABCDEF

    Host-to-Crazyflie: <port/chan> 0x02 0x01
    Crazyflie-to-Host: <port/chan> 0x02 0x01 0x01 0x70 0x00 0x00 0x00 0xEF 0xCD 0xAB 0x90 0x78 0x56 0x34 0x12

Example of requesting the information for a memory index that is not
valid

    Host-to-Crazyflie: <port/chan> 0x01 0x10
    Crazyflie-to-Host: <port/chan> 0x01 0x10

### SET\_MEM\_ERASE

This command is used to mass erase a memory with a given id. The id of
memories is sequential and is from 0 to one less than the returned
number of memories from GET\_NUMBER\_OF\_MEMS.

The request from host to Crazyflie:

|  Byte  | Field            | Value  | Length  | Comment|
|  ------| ------------------|-------|--------|--------------|
|  0     | SET\_MEM\_ERASE  | 0x03   | 1      |  The command byte|
|  1     | MEM\_ID          |        | 1      |  A memory id that is 0 \<= id \<| NBR\_OF\_MEMS

Reply from Crazyflie to host:

 | Byte  | Field            | Value  | Length  | Comment|
 | ------| -----------------| -------| --------| ---------------------------------------|
 | 0     | SET\_MEM\_ERASE  | 0x03   | 1       | The command byte|
 | 1     | MEM\_ID          |        | 1       | The memory id|
 | 2     | STATUS           |        | 1       | The status of the command (see below)|

Example of requesting a mass erase for a memory with MEM\_ID=2

    Host-to-Crazyflie: <port/chan>
    Crazyflie-to-Host: <port/chan>

Example of

    Host-to-Crazyflie: <port/chan>
    Crazyflie-to-Host: <port/chan>

Channel 1: Memory read
----------------------

This channel is only used to read memories and therefore the messages do
not contain any command byte.

The request from host to Crazyflie:

|  Byte  | Field      | Value   |Length   |Comment|
|  ------| -----------|  ------- -------- |-------------------------------------------------|
|  0     | MEM\_ID    |        | 1        |A memory id that is 0 \<= id \< NBR\_OF\_MEMS |
|  1     | MEM\_ADDR  |         |4        |The address where the first byte should be read |
|  5     | LEN         |        |1        |The number of bytes to be read |

Reply from Crazyflie to host if memory id is valid and the address and
length to be read is valid:

|  Byte |  Field    |  Value   |Length  | Comment|
|  -----| -----------| ------- |-------- |--------------|
|  0     | MEM\_ID   |         | 1       | A memory id that is 0 \<= id \< NBR\_OF\_MEMS|
|  1    |  MEM\_ADDR |         | 4       | The address where the first byte should be read|
|  5    |  STATUS    |         | 1       | The status of the request (see below)|
|  6    |  DATA      |         | 1..24?   |The data that was read (only if MEM\_ID/MEM\_ADDR/LEN is valid)|

Where the STATUS field is:

  STATUS   Comment
  -------- ---------
  0        \...
  1        \....

Example of reading LEN=0x0F bytes from MEM\_ID=0x01 MEM\_ADDR=0x0A

    Host-to-Crazyflie: <port/chan> 0x01 0x0A 0x00 0x00 0x00 0x0F
    Crazyflie-to-Host: <port/chan> 0x01 0x0A 0x00 0x00 0x00 0x00 0x01 0x09 0x62 0x63 0x4C 0x65 0x64 0x52 0x69 0x6E 0x67 0x02 0x01 0x62 0x55

Channel 2: Memory write
-----------------------

This channel is only used to write memories and therefore the messages
do not contain any command byte.

The request from host to Crazyflie:

 | Byte  | Field      | Value  | Length  | Comment|
|  ------| ----------- |------- |--------| -------------------------------------------------|
 | 0    |  MEM\_ID    |         |1       | A memory id that is 0 \<= id \< NBR\_OF\_MEMS|
 | 1    |  MEM\_ADDR   |       | 4       | The address where the first byte should be read|
 | 5    |  DATA        |       | 1..24?  | The data to be written|

Reply from Crazyflie to host if memory id is valid and the address and
length to be written is valid:

 | Byte  | Field      | Value  | Length  | Comment|
 | ------| -----------| -------| --------| -------------------------------------------------|
 | 0      |MEM\_ID    |       |  1      |  A memory id that is 0 \<= id \< NBR\_OF\_MEMS|
 | 1     | MEM\_ADDR  |       |  4      |  The address where the first byte should be read|
 | 4      |STATUS    |        |  1       | The status of the request (see below)|

Where the STATUS field is:

 | STATUS  | Comment|
 | --------| ---------|
 | 0      | \...|
 | 1      |  \....|

Example
