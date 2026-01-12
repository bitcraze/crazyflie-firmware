/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <stdbool.h>
#include <stddef.h>


#include "cfassert.h"
#include "crtp.h"
#include "supervisor.h"
#include "crtp_supervisor.h"
#include <string.h>
#include "debug.h"

static bool isInit = false;

static void crtpSupervisorCB(CRTPPacket* p);
static void handleSupervisorInfo(CRTPPacket* p);
static void handleSupervisorCommand(CRTPPacket* p);
static void sendSupervisorResponse(uint8_t channel, uint8_t cmd, const void* data, uint8_t len);


void crtpSupervisorInit(void)
{
    if(isInit) {
        return;
    }
    // Register the callback for the supervisor port
    crtpRegisterPortCB(CRTP_PORT_SUPERVISOR, crtpSupervisorCB);
    isInit = true;
}


static void crtpSupervisorCB(CRTPPacket* pk)
// It reads the channel from the packet and dispatches it to the correct handler.
{

    switch (pk->channel)
    {
        case SUPERVISOR_CH_INFO:
            handleSupervisorInfo(pk);
            break;

        case SUPERVISOR_CH_COMMAND:
            handleSupervisorCommand(pk);
            break;

        default:
            break;
    }

}


static void handleSupervisorInfo(CRTPPacket* p)
// It handles the requests to the Info channel.
{
    if (p->size < 1) {
        return;
    }

    uint8_t cmd = p->data[0];
    uint8_t value8 = 0;
    uint16_t bitfield = supervisorGetInfoBitfield();

    // Bit mapping: bit 0 = canBeArmed, bit 1 = isArmed, etc.
    switch (cmd)
    {
        case CMD_CAN_BE_ARMED:        value8 = (bitfield >> 0) & 0x01; break;
        case CMD_IS_ARMED:            value8 = (bitfield >> 1) & 0x01; break;
        case CMD_IS_AUTO_ARMED:       value8 = (bitfield >> 2) & 0x01; break;
        case CMD_CAN_FLY:             value8 = (bitfield >> 3) & 0x01; break;
        case CMD_IS_FLYING:           value8 = (bitfield >> 4) & 0x01; break;
        case CMD_IS_TUMBLED:          value8 = (bitfield >> 5) & 0x01; break;
        case CMD_IS_LOCKED:           value8 = (bitfield >> 6) & 0x01; break;
        case CMD_IS_CRASHED:          value8 = (bitfield >> 7) & 0x01; break;
        case CMD_HL_CONTROL_ACTIVE:   value8 = (bitfield >> 8) & 0x01; break;
        case CMD_HL_TRAJ_FINISHED:    value8 = (bitfield >> 9) & 0x01; break;
        case CMD_HL_CONTROL_DISABLED: value8 = (bitfield >> 10) & 0x01; break;

        case CMD_GET_STATE_BITFIELD:
            sendSupervisorResponse(SUPERVISOR_CH_INFO, cmd, &bitfield, sizeof(bitfield));
            return;

        default:
            return;
    }

    sendSupervisorResponse(SUPERVISOR_CH_INFO, cmd, &value8, 1);
}


static void handleSupervisorCommand(CRTPPacket* p)
{
    if (p->size < 1) {
        return;
    }

    uint8_t cmd = p->data[0];
    uint8_t* data = &p->data[1];

    switch (cmd) {

        case CMD_ARM_SYSTEM:
        {
            const bool doArm = data[0];
            const bool success = supervisorRequestArming(doArm);

            uint8_t resp[2];
            resp[0] = success;
            resp[1] = supervisorIsArmed();

            sendSupervisorResponse(SUPERVISOR_CH_COMMAND, cmd, resp, sizeof(resp));
            break;
        }

        case CMD_RECOVER_SYSTEM:
        {
            const bool success = supervisorRequestCrashRecovery(true);

            uint8_t resp[2];
            resp[0] = success;
            resp[1] = !supervisorIsCrashed();

            sendSupervisorResponse(SUPERVISOR_CH_COMMAND, cmd, resp, sizeof(resp));
            break;
        }

        default:
            break;
    }
}


static void sendSupervisorResponse(uint8_t channel, uint8_t cmd, const void* data, uint8_t len)
// Helper to send a response packet.
{
    CRTPPacket response;
    response.header = CRTP_HEADER(CRTP_PORT_SUPERVISOR, channel);
    response.size = 1 + len;

    response.data[0] = CMD_RESPONSE | cmd;
    memcpy(&response.data[1], data, len);

    crtpSendPacket(&response);
}