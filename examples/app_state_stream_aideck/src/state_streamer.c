/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 *
 * state_streamer.c - App layer application that streams the state estimate
 * of the Crazyflie to the GAP8 on the AI-deck, over UART1.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "crc32.h"
#include "uart1.h"
#include "log.h"

#define DEBUG_MODULE "SEND_STATE"
#include "debug.h"

#define STATE_STREAM_RATE_HZ 100

#define UART_STATE_MSG_HEADER "!STA"
typedef struct {
  uint8_t header[4];

  // STM32 timestamp - ticks
  uint32_t timestamp;

  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;

  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;

  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;

  // compressed quaternion, see quatcompress.h (elements stored as xyzw)
  int32_t quat;

  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;

  uint32_t checksum;
} __attribute__((packed)) state_msg_t;

// The message is static because a buffer sent with DMA must not live on the stack
static state_msg_t msg;

static void sendStateMsg(void) {
  memcpy(msg.header, UART_STATE_MSG_HEADER, sizeof(msg.header));
  msg.checksum = crc32CalculateBuffer(&msg, sizeof(msg) - sizeof(msg.checksum));
  uart1SendDmaIfAvailable(sizeof(msg), (uint8_t *)&msg);
}

void appMain() {
  uart1Init(115200);

  // The state is read from the compressed state estimate, which has the same fields
  // and units as the message. The values are read one by one and stamped when read,
  // so a message is not an exact snapshot of one estimator update.
  logVarId_t idX = logGetVarId("stateEstimateZ", "x");
  logVarId_t idY = logGetVarId("stateEstimateZ", "y");
  logVarId_t idZ = logGetVarId("stateEstimateZ", "z");
  logVarId_t idVx = logGetVarId("stateEstimateZ", "vx");
  logVarId_t idVy = logGetVarId("stateEstimateZ", "vy");
  logVarId_t idVz = logGetVarId("stateEstimateZ", "vz");
  logVarId_t idAx = logGetVarId("stateEstimateZ", "ax");
  logVarId_t idAy = logGetVarId("stateEstimateZ", "ay");
  logVarId_t idAz = logGetVarId("stateEstimateZ", "az");
  logVarId_t idQuat = logGetVarId("stateEstimateZ", "quat");
  logVarId_t idRateRoll = logGetVarId("stateEstimateZ", "rateRoll");
  logVarId_t idRatePitch = logGetVarId("stateEstimateZ", "ratePitch");
  logVarId_t idRateYaw = logGetVarId("stateEstimateZ", "rateYaw");

  TickType_t lastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(STATE_STREAM_RATE_HZ));

    msg.timestamp = xTaskGetTickCount();

    msg.x = logGetInt(idX);
    msg.y = logGetInt(idY);
    msg.z = logGetInt(idZ);

    msg.vx = logGetInt(idVx);
    msg.vy = logGetInt(idVy);
    msg.vz = logGetInt(idVz);

    msg.ax = logGetInt(idAx);
    msg.ay = logGetInt(idAy);
    msg.az = logGetInt(idAz);

    msg.quat = logGetUint(idQuat);

    msg.rateRoll = logGetInt(idRateRoll);
    msg.ratePitch = logGetInt(idRatePitch);
    msg.rateYaw = logGetInt(idRateYaw);

    sendStateMsg();
  }
}
