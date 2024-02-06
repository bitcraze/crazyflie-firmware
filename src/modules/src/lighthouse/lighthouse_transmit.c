/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * lighthouse_transmit.c - Transmit raw lighthouse data over radio
 */
#include "crtp.h"
#include "lighthouse_transmit.h"
#include "param.h"

#define DEBUG_MODULE "LH"
#include "debug.h"

typedef struct lighthouseTransmitState {
  uint8_t enabled;

  CRTPPacket txPacket;
  uint8_t queuedPackets;

  uint32_t last_timestamp;
  uint32_t ref_timestamp;
  uint32_t sensor_timestamps[4];
  uint32_t offset;
  int8_t channel;
  bool slowBit;
  int8_t sensor_flags;

} lighthouseTransmitState;

static lighthouseTransmitState gState = {
					 .enabled = 0,
	 .channel = -1,
	 .txPacket = {
		      .port = 6,
		      .channel = 3
         }
};

static void lighthouseResetCurrent(lighthouseTransmitState* state) {
  state->channel = -1;
  state->sensor_flags = 0;
  state->offset = 0;
  state->ref_timestamp = 0;
}

static void lighthouseTransmit(lighthouseTransmitState* state) {
  if(state->txPacket.size) {
    crtpSendPacket(&state->txPacket);
    state->txPacket.size = 0;
    state->queuedPackets = 0;
  }
}

static inline int32_t timediff_24bit(int32_t first_ts, int32_t second_ts) {
  int32_t diff = second_ts - first_ts;
  if(abs(diff) > 1 << 23) {
    diff -= 1 << 24;
  }
  return diff;
}

static void lighthouseQueue(lighthouseTransmitState* state) {
  if(state->channel == -1) return;

  // Basic strategy:
  // 3     Bytes: 0bCCCC 00OY 0xYY 0xYY - Channel, OOTX bit, 17 bits of sync data
  // 3     Bytes: 0xTT TT TT - 24mhz Timestamp of first packet
  // 2 * 4 Bytes: 0xOO OO - 2 Byte offset from timestamp per sensor
  // 14 bytes; so we do two at a time
  int offset = state->queuedPackets * 14;
  uint8_t* data = &state->txPacket.data[offset];
  uint32_t ref_timestamp = state->ref_timestamp;

  uint32_t channel_ootx_sync =
    ((uint32_t)state->channel << 20) |
    (state->slowBit << 17) |
    (state->offset / 4);

  memcpy(data, &channel_ootx_sync, 3);
  memcpy(&data[3], ((uint8_t*)&state->ref_timestamp), 3);

  if(state->offset > state->ref_timestamp) {
    state->offset -= (1 << 24);
  }

  for(int i = 0;i < 4;i++) {
    int16_t delta16 = 0x7FFF;
    if(state->sensor_flags & (1 << i)) {
      int32_t ts = state->sensor_timestamps[i];

      int32_t delta = ts - ref_timestamp;
      if(abs(delta) < 0x7FFF) {
	delta16 = delta;
      }
    }
    memcpy(&data[6 + 2 * i], &delta16, 2);
  }

  state->txPacket.size += 14;
  lighthouseResetCurrent(state);
  if(state->queuedPackets) {
    lighthouseTransmit(state);
  } else {
    state->queuedPackets = 1;
  }
}

void lighthouseTransmitProcessFrame(const lighthouseUartFrame_t* frame) {
  lighthouseTransmitState* state = &gState;
  if(!state->enabled) {
    return;
  }

  int32_t timediff = 0;
  if(frame->data.timestamp > state->ref_timestamp) timediff = frame->data.timestamp - state->last_timestamp;
  else if(frame->data.timestamp > state->ref_timestamp) timediff = state->last_timestamp - frame->data.timestamp;
  state->last_timestamp = frame->data.timestamp;

  if(state->channel != -1 && timediff > 0xFFFF) {
    lighthouseQueue(state);
  }

  uint32_t ts_mid = frame->data.timestamp + (frame->data.width / 2);
  if(frame->data.channelFound) {
    if(state->channel == -1) {
      state->channel = frame->data.channel;
      state->slowBit = frame->data.slowBit;
      state->ref_timestamp = frame->data.timestamp;
    } else if (state->channel != frame->data.channel) {
      lighthouseQueue(state);
      state->channel = frame->data.channel;
      state->ref_timestamp = frame->data.timestamp;
    }
  }

  if(frame->data.offset) {
    state->offset = frame->data.offset;
    state->ref_timestamp = frame->data.timestamp;
  }

  state->sensor_timestamps[frame->data.sensor] = ts_mid;
  state->sensor_flags |= 1 << frame->data.sensor;
  if(state->sensor_flags == 0xf) {
    lighthouseQueue(state);
  }

}
void lighthouseTransmitProcessTimeout() {
  lighthouseTransmitState* state = &gState;
  if(!state->enabled) {
    return;
  }
  lighthouseQueue(state);
  lighthouseTransmit(state);
}
PARAM_GROUP_START(lighthouse)
  PARAM_ADD(PARAM_UINT8, enLhRawStream, &gState.enabled)
PARAM_GROUP_STOP(lighthouse)
