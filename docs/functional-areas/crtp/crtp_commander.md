---
title: Commander
page_id: crtp_commander
---

The commander port is used to send control set-points for the
roll/pitch/yaw/thrust regulators from the host to the Crazyflie. As soon
as the communication link has been established these packets can be sent
and the values are valid until the next packet is received.

## Communication protocol

            +-------+-------+-------+-------+
            | ROLL  | PITCH |  YAW  |THRUST |
            +-------+-------+-------+-------+
    Length      4       4       4       2      bytes

|  Name    | Byte  |  Size  | Type       | Comment|
|  --------| -------| ------| -----------| ----------------------|
|  ROLL    | 0-3    | 4     | float      | The pitch set-point|
|  PITCH   | 4-7    | 4     | float      | The roll set-point|
|  YAW     | 8-11   | 4     | float      | The yaw set-point|
|  THRUST  | 12-13  | 2     | uint16\_t  | The thrust set-point|
