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
 * udplink_sim.h - CRTP link over UDP for Simmyflie (CONFIG_PLATFORM_SIM).
 *
 * Same shape as radiolinkGetLink()/usblinkGetLink() (see crtp.h), and the
 * same idea as CrazySim's socketlink.c, but Simmyflie plays the server role:
 * a client (cfclient/cflib/cfcli) connects to us, so the peer address is
 * learned from the first inbound packet rather than fixed at startup.
 *
 * Uses the UDP socket already bound by instanceInit() (instance_sim.h) --
 * no separate bind here.
 */
#ifndef UDPLINK_SIM_H_
#define UDPLINK_SIM_H_

#include "crtp.h"

/**
 * Starts the UDP RX task delivering packets into the CRTP link. The socket
 * must already be bound (instanceInit() before vTaskStartScheduler()).
 */
void udplinkInit(void);

/* Get the CRTP link operations for the UDP link (only link used in sim). */
struct crtpLinkOperations *udplinkGetLink(void);

#endif // UDPLINK_SIM_H_
