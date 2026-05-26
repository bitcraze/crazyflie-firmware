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
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "crc32.h"
#include "uart1.h"
#include "crtp_commander_high_level.h"
#include "stabilizer_types.h"
#include "log.h"

#define DEBUG_MODULE "SEND_STATE"
#include "debug.h"

#define UART_STATE_MSG_HEADER "!STA"
typedef struct {
  uint8_t header[4];
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

void send_state_msg(state_msg_t *msg) {
  memcpy(msg->header, UART_STATE_MSG_HEADER, sizeof(msg->header));
  msg->checksum = crc32CalculateBuffer(msg, sizeof(*msg) - sizeof(msg->checksum));
  uart1SendData(sizeof(state_msg_t), (uint8_t *)msg);
}

static void forwardState(const state_t *state) {
  
  state_msg_t msg = {
    .timestamp = usecTimestamp(),

    .x = state->position.x,
    .y = state->position.y,
    .z = state->position.z,

    .vx = state->velocity.x,
    .vy = state->velocity.y,
    .vz = state->velocity.z,

    .ax = state->acc.x,
    .ay = state->acc.y,
    .az = state->acc.z,

    // .quat = state->attitudeQuaternion,

    .rateRoll = state->attitude.roll,
    .ratePitch = state->attitude.pitch,
    .rateYaw = state->attitude.yaw,
  };
  send_state_msg(&msg);
}

void appMain() {
  uart1Init(115200);
  state_t current_state;
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  
  while(1) {
    vTaskDelay(M2T(100));
    
    current_state.position.x = logGetFloat(idX)*1000;
    current_state.position.y = logGetFloat(idY)*1000;
    current_state.position.z = logGetFloat(idZ)*1000;
    DEBUG_PRINT("Current state: %f %f %f\n", (double)current_state.position.x, (double)current_state.position.y, (double)current_state.position.z);
    forwardState(&current_state);

  }
}
