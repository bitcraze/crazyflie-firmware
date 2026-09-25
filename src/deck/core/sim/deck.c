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
 * deck.c (sim) - deck.h backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.9.
 *
 * The real deck_info.c tracks requiredKalmanEstimatorAttitudeReversionOff
 * as a flag ORed in from every discovered deck's memory-defined
 * requiredEstimator/estimation-hint fields -- deck discovery itself
 * (deck_discovery.c/deck_drivers.c) isn't built under sim (no deck bus
 * exists -- i2cdevInit() is a no-op stub, see i2cdev_sim.c/Phase 4.0), so
 * there is no deck that could ever request this. Fixed false forever: no
 * deck ever overrides attitude-reversion in sim.
 *
 * Also registers the "deck" param group. On real hardware every compiled-in
 * deck driver adds its own read-only bc<Deck> "is attached" param to this
 * group, and they exist (reading 0) even with no deck attached -- cfclient
 * relies on that (FlightTab, lighthouse_tab, LEDRingTab, ColorLEDTab index
 * param.values["deck"][...] unguarded). Since no driver is built under sim,
 * every bc* param declared by any driver in src/deck/drivers/src/ is
 * mirrored here, all backed by one constant 0 (no deck is ever attached).
 * CORE/non-CORE flags match each driver's own declaration. bcLoadcell is
 * declared by both loadcell.c and loadcell_nau7802.c and appears once.
 * Keep in sync when a driver adds or removes a bc* param.
 */

#include "sim/deck.h"
#include "param.h"

static uint8_t noDeckAttached = 0;

bool deckGetRequiredKalmanEstimatorAttitudeReversionOff(void)
{
  return false;
}

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBuzzer, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcCam, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCamLink, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedBot, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcColorLedTop, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &noDeckAttached)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &noDeckAttached)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &noDeckAttached)
PARAM_GROUP_STOP(deck)
