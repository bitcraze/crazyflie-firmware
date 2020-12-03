/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * appchanel_test.c: Demonstrate the appchanel functionality 
 */

#include "app.h"
#include "app_channel.h"

#include "debug.h"

#define DEBUG_MODULE "HELLOWORLD"

struct testPacketRX {
  float x;
  float y;
  float z;
} __attribute__((packed));

struct testPacketTX {
  float sum;
} __attribute__((packed));

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  struct testPacketRX rxPacket;
  struct testPacketTX txPacket;

  while(1) {
    if (appchannelReceivePacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {

      DEBUG_PRINT("App channel received x: %f, y: %f, z: %f\n", (double)rxPacket.x, (double)rxPacket.y, (double)rxPacket.z);

      txPacket.sum = rxPacket.x;
      txPacket.sum += rxPacket.y;
      txPacket.sum += rxPacket.z;

      appchannelSendPacket(&txPacket, sizeof(txPacket));
    }
  }
}