/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * crtp_control.h - CRTP port 10 bridge to the Control router.
 *
 * Lets a ground client list the Control endpoints and tunnel
 * bitcraze.control requests to them. See
 * docs/functional-areas/crtp/crtp_control.md.
 */

#ifndef CRTP_CONTROL_H_
#define CRTP_CONTROL_H_

#include <stdbool.h>

#ifdef UNIT_TEST_MODE
#include "crtp.h"
#endif

/** Register the CRTP port and start the bridge task. */
void crtpControlInit(void);

/** True after crtpControlInit(). */
bool crtpControlTest(void);

#ifdef UNIT_TEST_MODE
/** Handle one CRTP port 10 packet, as the bridge task does. */
void crtpControlHandlePacketForTest(CRTPPacket *packet);
/** Send or drop the responses of completed slots, as the bridge task does. */
void crtpControlFinishSlotsForTest(void);
/** Run the downlink step of the task: responses, notifications, losses. */
void crtpControlSendDownlinkForTest(void);
/** Reset bridge state between host tests. */
void crtpControlResetForTest(void);
#endif

#endif /* CRTP_CONTROL_H_ */
