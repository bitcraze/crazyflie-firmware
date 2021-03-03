/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2021 BitCraze AB
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
 * eventtrigger.c - Event triggers to mark important system events with payloads
 */
#include <string.h>

#include "eventtrigger.h"

#include "debug.h"

static eventtriggerCallback callbacks[eventtriggerHandler_Count] = {0};

/* Symbols set by the linker script */
extern eventtrigger _eventtrigger_start;
extern eventtrigger _eventtrigger_stop;

uint16_t eventtriggerGetId(const eventtrigger *event)
{
    // const eventtrigger* start = &_eventtrigger_start;
    return event - &_eventtrigger_start;
    return 0;
}

const eventtrigger *eventtriggerGetById(uint16_t id)
{
    const eventtrigger *result = &_eventtrigger_start;
    int numEventtriggers = &_eventtrigger_stop - &_eventtrigger_start;
    if (id < numEventtriggers) {
        return &result[id];
    }
    return 0;
}

const eventtrigger* eventtriggerGetByName(const char *name)
{
    const eventtrigger* result = &_eventtrigger_start;
    int numEventtriggers = &_eventtrigger_stop - &_eventtrigger_start;
    for (int i = 0; i < numEventtriggers; ++i) {
        if (strcmp(result[i].name, name) == 0) {
            return &result[i];
        }
    }
    return 0;
}

void eventTrigger(const eventtrigger *event)
{
    for (int i = 0; i < eventtriggerHandler_Count; ++i) {
        if (callbacks[i]) {
            callbacks[i](event);
        }
    }
}

void eventtriggerRegisterCallback(enum eventtriggerHandler_e handler, eventtriggerCallback cb)
{
    callbacks[handler] = cb;
}
