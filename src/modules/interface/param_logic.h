/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * param_logic.h - Crazy parameter system header file for the parameter logic.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <param.h>
#include <crtp.h>

/* Public API to access param variables */

/** Variable identifier.
 *
 * Should be fetched with paramGetVarId(). This is to be considered as an
 * opaque type, internal structure might change.
 *
 * Use PARAM_VARID_IS_VALID() to check if the ID is valid.
 */
typedef struct paramVarId_s {
  uint16_t id;
  uint16_t index;
} __attribute__((packed)) paramVarId_t;

/** Get the varId from group and name of variable
 *
 * @param group Group name of the variable
 * @param name Name of the variable
 * @return The variable ID or an invalid ID. Use PARAM_VARID_IS_VALID() to check validity.
 */
paramVarId_t paramGetVarId(const char* group, const char* name);

/** Get the varId from complete name of variable
 *
 * @param completeName Complete  name of the variable
 * @return The variable ID or an invalid ID. Use PARAM_VARID_IS_VALID() to check validity.
 */
paramVarId_t paramGetVarIdFromComplete(const char* completeName);

/** Check variable ID validity
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return true if the variable ID is valid, false otherwise.
 */
#define PARAM_VARID_IS_VALID(varId) (varId.id != 0xffffu)

/** Return the parameter type
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return Type of the variable. The value correspond to the defines used when
 *         declaring a param variable.
 */
int paramGetType(paramVarId_t varid);

/** Get group and name strings of a parameter
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @param group Pointer to a char* that will be filled with the group name
 * @param group Pointer to a char* that will be filled with the variable name
 *
 * The string buffers must be able to hold at least 32 bytes.
 */
void paramGetGroupAndName(paramVarId_t varid, char** group, char** name);

/** Get parameter variable size in byte
 *
 * @param type Type returned by paramGetType()
 * @return Size in byte occupied by variable of this type
 */
uint8_t paramVarSize(int type);

/** Return float value of a parameter
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
float paramGetFloat(paramVarId_t varid);

/** Return int value of a parameter
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
int paramGetInt(paramVarId_t varid);

/** Return Unsigned int value of a paramter
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return Current value of the variable
 */
unsigned int paramGetUint(paramVarId_t varid);

/** Set int value of an int parameter (1-4 bytes)
 *
 *  An update is also send to the client
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @param valuei Value to set in the variable
 */
void paramSetInt(paramVarId_t varid, int valuei);

/** Set float value of a float parameter
 *
 *  An update is also send to the client
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @param valuef Value to set in the variable
 */
void paramSetFloat(paramVarId_t varid, float valuef);

/**
 * @brief Initialize the parameter subsystem
 */
void paramLogicInit();

/**
 * @brief Read parameter values from persistent storage
 */
void paramLogicStorageInit();

// The following functions SHALL NOT be called outside paramTask!
void paramWriteProcess(CRTPPacket *p);
void paramReadProcess(CRTPPacket *p);
void paramTOCProcess(CRTPPacket *p, int command);

void paramGetDefaultValue(CRTPPacket *p);
void paramSetByName(CRTPPacket *p);
void paramGetExtendedType(CRTPPacket *p);
void paramPersistentStore(CRTPPacket *p);
void paramPersistentGetState(CRTPPacket *p);
void paramPersistentClear(CRTPPacket *p);
