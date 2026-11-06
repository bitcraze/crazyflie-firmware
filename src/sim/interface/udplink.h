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
 * udplink.h - CRTP link over UDP of the Simmyflie.
 *
 * Used in place of the radio link, with the same shape as
 * radiolinkGetLink() (see crtp.h). The Simmyflie is the server: a client
 * (cfclient, cflib) connects to it, so the peer address is learned from the
 * first packet that arrives.
 *
 * Uses the UDP socket bound by instanceInit() (instance.h).
 */
#ifndef UDPLINK_H_
#define UDPLINK_H_

#include "crtp.h"

/**
 * Starts the UDP RX task delivering packets into the CRTP link. The socket
 * must already be bound (instanceInit() before vTaskStartScheduler()).
 */
void udplinkInit(void);

/* Get the CRTP link operations for the UDP link (only link used in sim). */
struct crtpLinkOperations *udplinkGetLink(void);

#endif // UDPLINK_H_
