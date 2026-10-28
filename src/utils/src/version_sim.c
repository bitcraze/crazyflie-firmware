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
 * version_sim.c - version.h backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.2.
 *
 * The real version.c is generated at build time from version.vtpl via
 * tools/make/versionTemplate.py (git describe/rev-parse), and the template
 * itself includes "param.h" to expose firmware.revision0/1/modified as
 * real params -- the real param system isn't built until Phase 4.4. Fixed
 * placeholder strings here instead: platformservice.c's real
 * versionCommandProcess() (Phase 4.2, port 13 ch.1 getFirmwareVersion) is
 * the only current sim caller of V_STAG. Swap this file out for the real
 * generated version_gen.c once Phase 4.4 lands and PARAM_ADD_CORE() has a
 * real param system to register against.
 */

#include <stdbool.h>

const char * V_SLOCAL_REVISION = "0";
const char * V_SREVISION = "sim";
const char * V_STAG = "simmyflie";
const bool V_MODIFIED = false;
const bool V_PRODUCTION_RELEASE = false;
