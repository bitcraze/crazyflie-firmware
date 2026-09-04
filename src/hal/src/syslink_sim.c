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
 * syslink_sim.c - syslinkSendPacket() stub for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.2.
 *
 * The real syslink.c talks to the NRF51 companion radio chip over
 * uart_syslink.h/radiolink.h, neither of which exist in sim (same "NRF
 * companion chip doesn't exist in sim" reasoning Phase 4.0 used for
 * systemRequestNRFVersion()). platformservice.c's setContinuousWave
 * command is the only sim caller, forwarding a radio-test request that has
 * no meaning without real radio hardware -- stubbed to a no-op.
 */

#include <stdint.h>

#include "syslink.h"

int syslinkSendPacket(SyslinkPacket *slp)
{
  (void)slp;
  return 0;
}
