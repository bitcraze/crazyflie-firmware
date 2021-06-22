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
 * eventtrigger.h - Event triggers to mark important system events with payloads
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>


/* Data structures */

enum eventtriggerType_e {
    eventtriggerType_uint8  = 1,
    eventtriggerType_uint16 = 2,
    eventtriggerType_uint32 = 3,
    eventtriggerType_int8   = 4,
    eventtriggerType_int16  = 5,
    eventtriggerType_int32  = 6,
    eventtriggerType_float  = 7,
    eventtrigerType_fp16    = 8,
};
typedef float float_t;

typedef struct eventtriggerPayloadDesc_s
{
    enum eventtriggerType_e type;
    const char *name;
} eventtriggerPayloadDesc;

typedef struct eventtrigger_s
{
    const char *name;
    const struct eventtriggerPayloadDesc_s* payloadDesc;
    uint8_t numPayloadVariables;
    void *payload;
    uint8_t payloadSize;
} eventtrigger;

/* Example Use Case

static struct
{
  uint8_t var1;
  uint32_t var2;
} __attribute__((packed)) __eventTriggerPayload__myEvent__;

static const eventtriggerPayloadDesc __eventTriggerPayloadDesc__myEvent__[] = {
    {.type = eventtriggerType_uint8,
     .name = "var1"},
    {.type = eventtriggerType_uint32,
     .name = "var2"},
};

static const eventtrigger myEventTrigger = {
    .name = "myEvent",
    .payloadDesc = __eventTriggerPayloadDesc__myEvent__,
    .numPayloadVariables = sizeof(__eventTriggerPayloadDesc__myEvent__) / sizeof(eventtriggerPayloadDesc),
    .payload = &__eventTriggerPayload__myEvent__,
    .payloadSize = sizeof(__eventTriggerPayload__myEvent__),
};
*/

/* The same code above can be generated using the following macro:

EVENTTRIGGER(myEvent, uint8, var1, uint32, var2)

To debug/develop the macros, a good way is to create a new file "etdbg.c" with the following content
and then execute "gcc -E etdbg.c":

#include "src/modules/interface/eventtrigger.h"
EVENTTRIGGER(myEvent, uint8, var1, uint32, var2)
*/

#ifndef UNIT_TEST_MODE

/* Macro magic, see https://codecraft.co/2014/11/25/variadic-macros-tricks/ */
#define _GET_NTH_ARG(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, N, ...) N
#define _fe_0(_call, ...)
#define _fe_2(_call, k, v, ...) _call(k, v) _fe_0(_call, __VA_ARGS__)
#define _fe_4(_call, k, v, ...) _call(k, v) _fe_2(_call, __VA_ARGS__)
#define _fe_6(_call, k, v, ...) _call(k, v) _fe_4(_call, __VA_ARGS__)
#define _fe_8(_call, k, v, ...) _call(k, v) _fe_6(_call, __VA_ARGS__)
#define _fe_10(_call, k, v, ...) _call(k, v) _fe_8(_call, __VA_ARGS__)
#define CALL_MACRO_FOR_EACH_PAIR(x, ...)                \
    _GET_NTH_ARG("ignored", ##__VA_ARGS__,              \
    _fe_10, _invalid_,                                   \
    _fe_8, _invalid_,                                   \
    _fe_6, _invalid_,                                   \
    _fe_4, _invalid_,                                   \
    _fe_2, _invalid_,                                   \
    _fe_0)                                              \
    (x, ##__VA_ARGS__)
#define CALL_MACRO_IF_EMPTY(TRUE_MACRO, FALSE_MACRO, NAME, ...) \
    _GET_NTH_ARG("ignored", ##__VA_ARGS__,                      \
        TRUE_MACRO, _invalid_,                                  \
        TRUE_MACRO, _invalid_,                                  \
        TRUE_MACRO, _invalid_,                                  \
        TRUE_MACRO, _invalid_,                                  \
        TRUE_MACRO, _invalid_,                                  \
        FALSE_MACRO)(NAME, ##__VA_ARGS__)

#define _EVENTTRIGGER_ENTRY_PACKED(TYPE, NAME) \
    TYPE##_t NAME;

#define _EVENTTRIGGER_ENTRY_DESCRIPTION(TYPE, NAME) \
    {                                               \
        .type = eventtriggerType_##TYPE,            \
        .name = #NAME,                              \
    },

#define _EVENTTRIGGER_NON_EMPTY(NAME, ...)                                                                      \
    static struct                                                                                               \
    {                                                                                                           \
        CALL_MACRO_FOR_EACH_PAIR(_EVENTTRIGGER_ENTRY_PACKED, ##__VA_ARGS__)                                     \
    } __attribute__((packed)) eventTrigger_##NAME##_payload;                                                    \
    static const eventtriggerPayloadDesc __eventTriggerPayloadDesc__##NAME##__[] =                              \
        {                                                                                                       \
            CALL_MACRO_FOR_EACH_PAIR(_EVENTTRIGGER_ENTRY_DESCRIPTION, ##__VA_ARGS__)};                          \
    static const eventtrigger eventTrigger_##NAME __attribute__((section(".eventtrigger." #NAME), used)) = {    \
        .name = #NAME,                                                                                          \
        .payloadDesc = __eventTriggerPayloadDesc__##NAME##__,                                                   \
        .numPayloadVariables = sizeof(__eventTriggerPayloadDesc__##NAME##__) /                                  \
                               sizeof(eventtriggerPayloadDesc),                                                 \
        .payload = &eventTrigger_##NAME##_payload,                                                              \
        .payloadSize = sizeof(eventTrigger_##NAME##_payload),                                                   \
    };

#define _EVENTTRIGGER_EMPTY(NAME)                                                                               \
    static const eventtrigger eventTrigger_##NAME __attribute__((section(".eventtrigger." #NAME), used)) = {    \
        .name = #NAME,                                                                                          \
        .payloadDesc = NULL,                                                                                    \
        .numPayloadVariables = 0,                                                                               \
        .payload = NULL,                                                                                        \
        .payloadSize = 0,                                                                                       \
    };

#define EVENTTRIGGER(NAME, ...) \
    CALL_MACRO_IF_EMPTY(_EVENTTRIGGER_NON_EMPTY, _EVENTTRIGGER_EMPTY, NAME, ##__VA_ARGS__)

#else // UNIT_TEST_MODE

// Empty defines when running unit tests
#define EVENTTRIGGER(NAME, ...)

#endif // UNIT_TEST_MODE

/* Functions and associated data structures */

typedef void (*eventtriggerCallback)(const eventtrigger *);

enum eventtriggerHandler_e
{
    eventtriggerHandler_USD = 0,
    eventtriggerHandler_Count
};

/** Get the eventtrigger id from a pointer
 * 
 * @param event Pointer to the event
 * @return A unique id 0...numEvents-1 for the event
 */
uint16_t eventtriggerGetId(const eventtrigger *event);

/** Get the eventtrigger pointer from an event id
 * 
 * @param id unique id of the event
 * @return A pointer to the eventtrigger data structure, or NULL if no such eventtrigger exists
 */
const eventtrigger* eventtriggerGetById(uint16_t id);

/** Get the eventtrigger pointer from an event name
 * 
 * @param name Name of the event
 * @return A pointer to the eventtrigger data structure, or NULL if no such eventtrigger exists
 */
const eventtrigger* eventtriggerGetByName(const char *name);

/** Trigger the specified event
 * 
 * @param event Pointer to the event with updated payload
 * 
 * event->payload should be filled beforehand with metadata about the event
 */
void eventTrigger(const eventtrigger *event);

/** Register a callback that will be called for enabled events
 * 
 * @param handler unique type of the handler that this callback is for
 * @param cb function pointer to the callback
 * 
 * The handler allows multiple event handlers to be triggered by the same event.
 * The callback is only called for enabled events.
 */
void eventtriggerRegisterCallback(enum eventtriggerHandler_e handler, eventtriggerCallback cb);
