---
title: Supervisor CRTP port
page_id: crtp_supervisor
---

This port exposes supervisor commands and state queries.

## CRTP channels

| Port | Channel | Function |
|------|---------|----------|
| 9    | 0       | [State info](#state-info) |
| 9    | 1       | [Commands](#commands) |

Packets on channel 1 (commands) are handled in a high-priority CRTP callback. Packets on channel 0 (info) are handled in a lower-priority task.

For responses, byte 0 is the command ID with the high bit set (`cmd | 0x80`).

## Commands

The first byte of the payload is the command ID.

| Value | Command |
|-------|---------|
| 0x01  | [Arm/disarm system](#armdisarm-system) |
| 0x02  | [Recover system](#recover-system) |
| 0x03  | [Emergency stop](#emergency-stop) |
| 0x04  | [Emergency stop watchdog](#emergency-stop-watchdog) |

### Arm/disarm system

See [arming](/docs/functional-areas/supervisor/arming.md) for details.

Command:

| Byte | Description |
|------|-------------|
| 0    | command (0x01) |
| 1    | 0 = disarm, non-zero = arm |

Response:

| Byte | Description |
|------|-------------|
| 0    | command \| 0x80 |
| 1    | success: 1 if the requested arming state was set |
| 2    | isArmed: 0 = disarmed, 1 = armed |

### Recover system

Request crash recovery. Only succeeds if the Crazyflie is no longer tumbled. See [crash handling](/docs/functional-areas/supervisor/crash_handling.md) for details.

Command:

| Byte | Description |
|------|-------------|
| 0    | command (0x02) |

Response:

| Byte | Description |
|------|-------------|
| 0    | command \| 0x80 |
| 1    | success: 1 if crash recovery was accepted |
| 2    | isRecovered: 1 if no longer in crashed state |

### Emergency stop

Immediately stops all motors. The emergency stop state is latching and persists until reboot.

Command:

| Byte | Description |
|------|-------------|
| 0    | command (0x03) |

No response is sent.

### Emergency stop watchdog

Keepalive packet that must be sent at least once per second after the first packet is received. If the timeout expires, all motors are stopped. The watchdog is disabled at startup and activates on the first received packet.

Command:

| Byte | Description |
|------|-------------|
| 0    | command (0x04) |

No response is sent.

## State info

The first byte of the payload is the command ID. All responses echo the command ID with the high bit set (`cmd | 0x80`).

| Value | Command |
|-------|---------|
| 0x01  | canBeArmed |
| 0x02  | isArmed |
| 0x03  | isAutoArmed |
| 0x04  | canFly |
| 0x05  | isFlying |
| 0x06  | isTumbled |
| 0x07  | isLocked |
| 0x08  | isCrashed |
| 0x09  | hlControlActive |
| 0x0A  | hlTrajFinished |
| 0x0B  | hlControlDisabled |
| 0x0C  | [Get state bitfield](#get-state-bitfield) |

For commands 0x01–0x0B, the response payload is a single byte (0 or 1).

### Get state bitfield

Returns all state flags packed into a 16-bit value. Bit positions correspond to the command IDs above (bit 0 = canBeArmed, bit 1 = isArmed, etc.).

Command:

| Byte | Description |
|------|-------------|
| 0    | command (0x0C) |

Response:

| Byte | Description |
|------|-------------|
| 0    | command \| 0x80 |
| 1–2  | 16-bit state bitfield (little-endian) |
