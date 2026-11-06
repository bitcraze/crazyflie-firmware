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
 * version.c - Replacement of version.h for the Simmyflie.
 *
 * The mainline version.c is generated from version.vtpl with the git
 * revision, and registers the firmware.* params. The sim reports fixed
 * values, with "simmyflie" as the tag, and has no firmware.* params.
 */

#include <stdbool.h>

const char * V_SLOCAL_REVISION = "0";
const char * V_SREVISION = "sim";
const char * V_STAG = "simmyflie";
const bool V_MODIFIED = false;
const bool V_PRODUCTION_RELEASE = false;
