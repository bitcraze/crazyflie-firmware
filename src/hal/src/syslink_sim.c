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
 * syslink_sim.c - syslink stubs for CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Syslink is the UART protocol to the NRF51 radio chip, which the sim does
 * not have. Packets sent to it are dropped: system.c's NRF version request
 * and radio-ready notification, and platformservice.c's setContinuousWave.
 * Nothing ever arrives from it, so enabling incoming packets does nothing.
 */

#include <stdint.h>

#include "syslink.h"
#include "uart_syslink.h"

int syslinkSendPacket(SyslinkPacket *slp)
{
  (void)slp;
  return 0;
}

void uartslkEnableIncoming()
{
}
