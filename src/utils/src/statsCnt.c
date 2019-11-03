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
 * statsCnt.c - utitlity for logging counters
 */

#include "statsCnt.h"

void statsCntReset(statsCntRate_t* this, const uint32_t now_ms) {
    this->count = 0;
    this->result = 0.0f;
    this->latestAverage_ms = now_ms;
}

void statsCntInc(statsCntRate_t* this) {
    this->count++;
}

float statsCntRate(statsCntRate_t* this, const uint32_t now_ms) {
    float result = 0.0f;

    const uint32_t dt_ms = now_ms - this->latestAverage_ms;
    if (dt_ms > 0) {
        const float oneSecond_ms = 1000.0f;
        result = this->count * oneSecond_ms / dt_ms;
    }

    this->count = 0;
    this->result = result;
    this->latestAverage_ms = now_ms;

    return result;
}
