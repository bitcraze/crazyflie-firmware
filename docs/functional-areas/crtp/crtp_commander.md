---
title: Commander
page_id: crtp_commander
---

The commander port is used to send control set-points for the
roll/pitch/yaw/thrust regulators from the host to the Crazyflie. As soon
as the communication link has been established these packets can be sent
and the values are valid until the next packet is received. The packets
are decoded in `src/modules/src/crtp_commander_rpyt.c`.

## Communication protocol

            +-------+-------+-------+-------+
            | ROLL  | PITCH |  YAW  |THRUST |
            +-------+-------+-------+-------+
    Length      4       4       4       2      bytes

|  Name    | Byte  |  Size  | Type       | Comment|
|  --------| -------| ------| -----------| ----------------------|
|  ROLL    | 0-3    | 4     | float      | Roll set-point, degrees (angle mode) or degrees/s (rate mode)|
|  PITCH   | 4-7    | 4     | float      | Pitch set-point, degrees (angle mode) or degrees/s (rate mode)|
|  YAW     | 8-11   | 4     | float      | Yaw set-point, degrees/s (rate mode, default) or degrees (angle mode)|
|  THRUST  | 12-13  | 2     | uint16\_t  | Thrust set-point, 0 - 65535|

In rate mode the yaw value is negated on reception, so a positive value
rotates the Crazyflie clockwise seen from above.

## Thrust

The thrust field is passed to the controller as a 16 bit value. With
`CONFIG_ENABLE_THRUST_BAT_COMPENSATED` enabled it is linear in force: a
motor commanded with value `t` produces `t / UINT16_MAX * THRUST_MAX`
newtons (see `platform_defaults_cf2.h`), independent of battery voltage.
Without battery compensation the value is used directly as PWM duty
cycle, which is not linear in thrust and dependent on battery voltage.

* **Thrust lock.** After connecting, and whenever the commander priority
  is disabled, non-zero thrust is ignored until a packet with thrust 0
  has been received. This prevents the motors from starting on a stale
  or unintended set-point.
* **Lower bound.** Thrust values below `MIN_THRUST` are treated as zero. 
  If `CONFIG_ENABLE_THRUST_BAT_COMPENSATED` is enabled (default), the limit 
  is computed by `THRUST_MIN / THRUST_MAX * UINT16_MAX`.
* **Upper bound.** There is no clipping in the commander. The per-motor
  output is capped to the PWM range in the power distribution.

## Flight modes

The `flightmode` parameter group changes how the four fields are
interpreted.

| Parameter | Effect |
| --------- | ------ |
| `stabModeRoll`, `stabModePitch`, `stabModeYaw` | 0: rate control, 1: angle control. Defaults are angle for roll/pitch and rate for yaw. |
| `althold` | Thrust becomes a vertical velocity set-point centred at 32767; roll/pitch/yaw unchanged. |
| `poshold` | Roll and pitch become Y and X velocity set-points (value / 30 m/s); thrust as above. |
| `posSet` | Roll, pitch and thrust become absolute Y, X (m) and Z (thrust / 1000 m) positions, yaw becomes an absolute angle. |
| `yawMode` | Frame used for roll/pitch in yaw rate mode: 1 plus-mode, 2 x-mode (default). |

## Example

**[ramp.py](https://github.com/bitcraze/crazyflie-demos/tree/main/demos/scripts/cflib/motors/ramp)**: Sends low-level roll/pitch/yaw/thrust set-points directly using the Commander class, ramping thrust up and down.

Set-points must be sent continuously, see the watchdog timeouts in the
[commander framework](/docs/functional-areas/sensor-to-control/commanders_setpoints.md).
