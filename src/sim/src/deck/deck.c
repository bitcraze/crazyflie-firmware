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
 * deck.c - Replacement of deck.h for the Simmyflie.
 *
 * The sim has no deck bus and so no decks: nothing to initialize, no deck
 * requests an estimator, a radio mode or a change to the Kalman filter, and
 * deckSupervisorHasFault() never finds a fault.
 *
 * It also registers the "deck" param group. On a Crazyflie every deck driver
 * that is built adds its read-only bc<Deck> param to this group, which reads
 * 0 when that deck is not attached. cfclient reads several of them without
 * checking that they exist. No deck driver is built for the sim, so every
 * bc* param declared in src/deck/drivers/src/ is declared here, all reading
 * 0. The CORE flags follow each driver's declaration. bcLoadcell is declared
 * by both loadcell.c and loadcell_nau7802.c and appears once. Keep the list
 * in step with the drivers.
 */

#include "deck.h"
#include "deck_supervisor.h"
#include "param.h"

static uint8_t noDeckAttached = 0;

/* system.c's deck calls. With no deck there is nothing to initialize or
 * test, and no deck asks for an estimator or a radio mode. AutoSelect makes
 * stabilizerInit() fall back to the Kconfig default estimator. */
void deckInit(void)
{
}

bool deckTest(void)
{
  return true;
}

StateEstimatorType deckGetRequiredEstimator(void)
{
  return StateEstimatorTypeAutoSelect;
}

bool deckGetRequiredLowInterferenceRadioMode(void)
{
  return false;
}

bool deckGetRequiredKalmanEstimatorAttitudeReversionOff(void)
{
  return false;
}

bool deckSupervisorHasFault(void)
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
