/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _CRTP_SUPERVISOR_H_
#define _CRTP_SUPERVISOR_H_

#include "crtp.h"
#include <stdint.h>

#define SUPERVISOR_CH_INFO    0
#define SUPERVISOR_CH_COMMAND 1

// Commands
#define CMD_ARM_SYSTEM         0x01
#define CMD_RECOVER_SYSTEM     0x02

// State info
#define CMD_CAN_BE_ARMED          0x01
#define CMD_IS_ARMED              0x02
#define CMD_IS_AUTO_ARMED         0x03
#define CMD_CAN_FLY               0x04
#define CMD_IS_FLYING             0x05
#define CMD_IS_TUMBLED            0x06
#define CMD_IS_LOCKED             0x07
#define CMD_IS_CRASHED            0x08
#define CMD_HL_CONTROL_ACTIVE     0x09
#define CMD_HL_TRAJ_FINISHED      0x0A
#define CMD_HL_CONTROL_DISABLED   0x0B
#define CMD_GET_STATE_BITFIELD    0x0C

#define CMD_RESPONSE        0x80

void crtpSupervisorInit(void);

#endif