/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "LEDEFFECTS"

// Pack WRGB into 0xWWRRGGBB
#define WRGB(w, r, g, b)  ( ((uint32_t)(w) << 24) | ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b) )

#define COLOR_OFF WRGB(0, 0, 0, 0)

static uint8_t effect = 0;
static logVarId_t idX, idY, idZ;
static logVarId_t idVx, idVy, idVz;

static uint32_t prevWrgbBot = 0xFFFFFFFF;
static uint32_t prevWrgbTop = 0xFFFFFFFF;

//----------------Effect1: LED Cycle-------------

//------------------------------------------------

//----------------Effect2: Color Mapping---------
// Flight Space
#define MIN_X_BOUND -2.0f
#define MAX_X_BOUND  2.0f
#define MIN_Y_BOUND -2.0f
#define MAX_Y_BOUND  2.0f
#define MIN_Z_BOUND  0.0f
#define MAX_Z_BOUND  3.0f
//------------------------------------------------

//----------------Effect3: Velocity Indicator----
#define MAX_VEL 1.0f   // m/s, adjust if needed

#define COLOR_VEL_MIN WRGB(0,  0,   0, 255)   // Blue
#define COLOR_VEL_MID WRGB(0,  0, 255,   0)   // Green
#define COLOR_VEL_MAX WRGB(0,255,   0,   0)   // Red
//------------------------------------------------

//----------------Effect4: Snowflake-------------
#define LANDING_Z 0.0f
#define FADE_PERIOD_MS 2000
//------------------------------------------------

//----------------Effect5: Flicker---------------
#define COLOR_GOLD WRGB(0, 255, 156, 0)
// Define limits for off duration (in ms)
#define FLICKER_MIN_OFF 200
#define FLICKER_MAX_OFF 600
// State tracking
static TickType_t nextFlickerTop = 0;
static TickType_t nextFlickerBot = 0;
static uint8_t deckTopOn  = 1;
static uint8_t deckBotOn  = 1;
//------------------------------------------------


// Helper function to update WRGB parameters safely
static void updateDeckParamIfChanged(paramVarId_t id, uint32_t newValue, uint32_t *prevValue)
{
    if (id.id != 0xffffu && newValue != *prevValue) {
        paramSetInt(id, newValue);
        *prevValue = newValue;
    }
}


void appMain()
{
    DEBUG_PRINT("Starting WRGB color effects app...\n");

    // Detect LED decks
    paramVarId_t idBottomDetect = paramGetVarId("deck", "bcColorLedBot");
    paramVarId_t idTopDetect    = paramGetVarId("deck", "bcColorLedTop");

    uint8_t bottomAttached = paramGetUint(idBottomDetect);
    uint8_t topAttached    = paramGetUint(idTopDetect);

    // Deck WRGB param IDs
    paramVarId_t idWrgbBot = {0};
    paramVarId_t idWrgbTop = {0};

    // Thermal log IDs
    logVarId_t idDeckTemp    = {0}, idThrottlePct = {0};

    // Brightness correction
    if (bottomAttached) {
        idWrgbBot = paramGetVarId("colorLedBot", "wrgb8888");
        idDeckTemp    = logGetVarId("colorLedBot", "deckTemp");
        idThrottlePct = logGetVarId("colorLedBot", "throttlePct");
        paramSetInt(paramGetVarId("colorLedBot", "brightCorr"), 1);
        DEBUG_PRINT("Color LED Bottom deck detected\n");
    }
    if (topAttached) {
        idWrgbTop = paramGetVarId("colorLedTop", "wrgb8888");
        idDeckTemp    = logGetVarId("colorLedTop", "deckTemp");
        idThrottlePct = logGetVarId("colorLedTop", "throttlePct");
        paramSetInt(paramGetVarId("colorLedTop", "brightCorr"), 1);
        DEBUG_PRINT("Color LED Top deck detected\n");
    }

    uint8_t r = 0, g = 0, b = 0, w = 0;
    int step = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t lastThermalCheck = xTaskGetTickCount();
    const uint32_t thermalCheckInterval = M2T(100);

    /* Track previous effect to detect transitions */
    uint8_t prevEffect = effect; // initialize to current value so we don't clear immediately

    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");

    idVx = logGetVarId("stateEstimate", "vx");
    idVy = logGetVarId("stateEstimate", "vy");
    idVz = logGetVarId("stateEstimate", "vz");

    while (1)
    {
        /* If the parameter was changed since last iteration, handle the transition */
        if (prevEffect != effect) {
            DEBUG_PRINT("colorLED.effect changed %d -> %d\n", prevEffect, effect);

            // Clear prevWrgb to force updates in the new effect
            prevWrgbBot = 0xFFFFFFFF;
            prevWrgbTop = 0xFFFFFFFF;

            // If we just switched into effect == 0, clear the LEDs once.
            if (effect == 0) {
            if (bottomAttached) paramSetInt(idWrgbBot, COLOR_OFF);
            if (topAttached)    paramSetInt(idWrgbTop, COLOR_OFF);
            }

            prevEffect = effect;
    }

    if (effect == 1)
    {
        //---------------------------------------------------------
        // LED CYCLE
        //---------------------------------------------------------
        int phase = step / 256;
        int value = step % 256;

        switch(phase) {
        case 0:  // G: 0->255, R: 255->0
            r = 255 - value;
            g = value;
            b = 0;
            w = 0;
            break;

        case 1:  // B: 0->255, G: 255->0
            r = 0;
            g = 255 - value;
            b = value;
            w = 0;
            break;

        case 2:  // W: 0->255, B: 255->0
            r = 0;
            g = 0;
            b = 255 - value;
            w = value;
            break;

        case 3: // R: 0->255, W: 255->0
            r = value;
            g = 0;
            b = 0;
            w = 255 - value;
            break;
        }

        uint32_t wrgb_value = WRGB(w,r,g,b);

        // Set both decks to same value
        if (bottomAttached) paramSetInt(idWrgbBot, wrgb_value);
        if (topAttached)    paramSetInt(idWrgbTop, wrgb_value);

        step = (step + 1) % (256 * 4);  // original behavior
    }
    else if (effect == 2)
    {
        //---------------------------------------------------------
        // COLOR MAPPING
        //---------------------------------------------------------
        float x = logGetFloat(idX);
        float y = logGetFloat(idY);
        float z = logGetFloat(idZ);

        // Clamp position within bounds
        if (x < MIN_X_BOUND) x = MIN_X_BOUND;
        if (x > MAX_X_BOUND) x = MAX_X_BOUND;

        if (y < MIN_Y_BOUND) y = MIN_Y_BOUND;
        if (y > MAX_Y_BOUND) y = MAX_Y_BOUND;

        if (z < MIN_Z_BOUND) z = MIN_Z_BOUND;
        if (z > MAX_Z_BOUND) z = MAX_Z_BOUND;

        // Map X/Y/Z into 0–255 based on bounds
        float rf = ((x - MIN_X_BOUND) / (MAX_X_BOUND - MIN_X_BOUND)) * 255.0f;
        float gf = ((y - MIN_Y_BOUND) / (MAX_Y_BOUND - MIN_Y_BOUND)) * 255.0f;
        float bf = ((z - MIN_Z_BOUND) / (MAX_Z_BOUND - MIN_Z_BOUND)) * 255.0f;

        // Remove most of the white component (smallest of RGB)
        float removeWhite = rf;
        if (gf < removeWhite) removeWhite = gf;
        if (bf < removeWhite) removeWhite = bf;

        rf -= removeWhite * 0.8f;
        gf -= removeWhite * 0.8f;
        bf -= removeWhite * 0.8f;

        // Normalize RGB so sum ~ 255
        float sum = rf + gf + bf;
        if (sum < 1.0f) sum = 1.0f; // avoid div by zero

        uint8_t r = (uint8_t)(rf * 255.0f / sum);
        uint8_t g = (uint8_t)(gf * 255.0f / sum);
        uint8_t b = (uint8_t)(bf * 255.0f / sum);
        uint8_t w = 0; // keep white off

        uint32_t wrgb_value = WRGB(w,r,g,b);

        // Set both decks to same value
        if (bottomAttached) paramSetInt(idWrgbBot, wrgb_value);
        if (topAttached)    paramSetInt(idWrgbTop, wrgb_value);
    }
    else if (effect == 3)
    {
        //---------------------------------------------------------
        // VELOCITY COLOR INDICATOR
        //---------------------------------------------------------
        float vx = logGetFloat(idVx);
        float vy = logGetFloat(idVy);
        float vz = logGetFloat(idVz);

        float vel = sqrtf(vx*vx + vy*vy + vz*vz);

        // Clamp 0 -> MAX_VEL
        if (vel < 0) vel = 0;
        if (vel > MAX_VEL) vel = MAX_VEL;

        float t = vel / MAX_VEL;   // 0->1 normalized speed

        // Smooth gradient Blue -> Green -> Red
        uint8_t r, g, b;

        if (t < 0.5f) {
            // Blue -> Green (0–50%)
            float k = t / 0.5f;
            r = 0;
            g = (uint8_t)(255.0f * k);
            b = (uint8_t)(255.0f * (1.0f - k));
        } else {
            // Green -> Red (50–100%)
            float k = (t - 0.5f) / 0.5f;
            r = (uint8_t)(255.0f * k);
            g = (uint8_t)(255.0f * (1.0f - k));
            b = 0;
        }

        uint32_t wrgb_value = WRGB(0, r, g, b);

        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_value, &prevWrgbTop);
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_value, &prevWrgbBot);
    }
    else if (effect == 4)
    {
        //---------------------------------------------------------
        // SNOWFLAKE
        //---------------------------------------------------------
        // Capture max height when effect is activated
        static float max_height = 0.0f;
        static uint8_t effectPrev = 0;
        if (effectPrev != 4) {
            max_height = logGetFloat(idZ);  // current height becomes max brightness
        }
        effectPrev = 4;

        // Get current height
        float z = logGetFloat(idZ);

        // Clamp for safety
        if (z < LANDING_Z) z = LANDING_Z;
        if (z > max_height) z = max_height;

        // Linear scaling: LANDING_Z -> 0, max_height -> 1
        float heightFactor = (z - LANDING_Z) / (max_height - LANDING_Z);

        // Sinusoidal fade: smooth pulsing
        TickType_t t = xTaskGetTickCount();
        float fade = 0.5f * (1.0f + sinf(2.0f * 3.14159f * t * portTICK_PERIOD_MS / FADE_PERIOD_MS));

        uint8_t white = (uint8_t)(255.0f * heightFactor * fade);

        // Prepare WRGB value (R=G=B=0)
        uint32_t wrgb_value = ((uint32_t)white << 24) | 0x00000000;

        // Update decks safely
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_value, &prevWrgbBot);
        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_value, &prevWrgbTop);
    }
    else if (effect == 5)
    {
        //---------------------------------------------------------
        // FLICKER
        //---------------------------------------------------------
        uint32_t wrgb_value = COLOR_GOLD;
        uint32_t wrgb_off   = COLOR_OFF;

        TickType_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // --- Top deck ---
        if (t >= nextFlickerTop)
        {
            deckTopOn = !deckTopOn;  // toggle on/off
            // Randomize next toggle: simple pseudo-random based on tick
            uint32_t dur = FLICKER_MIN_OFF + ((t * 37) & (FLICKER_MAX_OFF - FLICKER_MIN_OFF));
            nextFlickerTop = t + dur;
        }

        // --- Bottom deck ---
        if (t >= nextFlickerBot)
        {
            deckBotOn = !deckBotOn;  // toggle on/off
            uint32_t dur = FLICKER_MIN_OFF + ((t * 53) & (FLICKER_MAX_OFF - FLICKER_MIN_OFF));
            nextFlickerBot = t + dur;
        }

        uint32_t wrgb_top  = deckTopOn  ? wrgb_value : wrgb_off;
        uint32_t wrgb_bot  = deckBotOn  ? wrgb_value : wrgb_off;

        if (topAttached)    updateDeckParamIfChanged(idWrgbTop, wrgb_top, &prevWrgbTop);
        if (bottomAttached) updateDeckParamIfChanged(idWrgbBot, wrgb_bot, &prevWrgbBot);
    }
    else
    {
        //---------------------------------------------------------
        // effect == 0 -> DO NOTHING
        // This allows the deck's own parameters to control the LEDs.
        //---------------------------------------------------------
    }

    // Thermal throttling log
    if (xTaskGetTickCount() - lastThermalCheck >= thermalCheckInterval) {
        uint8_t throttlePct = logGetUint(idThrottlePct);
        if (throttlePct) {
        uint8_t deckTemp = logGetUint(idDeckTemp);
        DEBUG_PRINT("WARNING: Thermal throttling active! Temp: %d°C, Throttle: %d%%\n",
                    deckTemp, throttlePct);
        }
        lastThermalCheck = xTaskGetTickCount();
    }

    vTaskDelayUntil(&lastWakeTime, M2T(10));
    }
}

// -------- PARAMETERS --------
PARAM_GROUP_START(colorLED)
PARAM_ADD(PARAM_UINT8, effect, &effect)
PARAM_GROUP_STOP(colorLED)
// ----------------------------
