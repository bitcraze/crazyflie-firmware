/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * msp.c: Implementation of the MultiWii Serial Protocol
 *
 * The MultiWii Serial Protocol is a commonly used protocol for OSDs,
 * data logging, and communication with a ground telemetry station.
 * For more details, see the official MSP documentation and support thread:
 *  - http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
 *  - http://www.multiwii.com/forum/viewtopic.php?f=8&t=1516
 */

#include "msp.h"
#include "debug.h"
#include "sensfusion6.h"
#include "commander.h"
#include "version.h"
#include "motors.h"
#include "string.h"
#include "platform.h"
#include "param.h"

// MSP command IDs
typedef enum {
  MSP_API_VERSION    = 1,
  MSP_FC_VARIANT     = 2,
  MSP_FC_VERSION     = 3,
  MSP_BOARD_INFO     = 4,
  MSP_BUILD_INFO     = 5,
  MSP_FEATURE_CONFIG = 36,
  MSP_STATUS         = 101,
  MSP_MOTOR          = 104,
  MSP_RC             = 105,
  MSP_ATTITUDE       = 108,
  MSP_BOXIDS         = 119,
  MSP_BATTERY_STATE  = 130,
  MSP_UID            = 160, // Unique device ID
  MSP_SET_MOTOR      = 214,
  MSP_SET_4WAY_IF    = 245
} msp_command_t;

// Misc MSP header defines
#define MSP_PREAMBLE_0    '$'
#define MSP_PREAMBLE_1    'M'
#define MSP_DIRECTION_IN  '<'
#define MSP_DIRECTION_OUT '>'

// Sensor flag defines
#define BARO_PRESENT  (0x01)
#define MAG_PRESENT   (0x02)
#define GPS_PRESENT   (0x04)
#define SONAR_PRESENT (0x08)

typedef enum
{
  MSP_REQUEST_STATE_WAIT_FOR_START,
  MSP_REQUEST_STATE_PREAMBLE,
  MSP_REQUEST_STATE_DIRECTION,
  MSP_REQUEST_STATE_SIZE,
  MSP_REQUEST_STATE_COMMAND,
  MSP_REQUEST_STATE_DATA,
  MSP_REQUEST_STATE_CRC
} MSP_REQUEST_STATE;

/**
 * Structures describing data payloads for MSP responses
 * All data must be serialized (LSB first) and the cflie
 * is using a little-endian processor, so no byte swapping
 * required
 */
typedef struct _MspStatus
{
  uint16_t cycleTime; // Rate of main stabilizer loop
  uint16_t i2cErrors; // Number of I2C bus errors encountered
  uint16_t sensors;   // Bitmask of supported sensors (see sensor flag bitmasks)
  uint32_t flags;     // Bitmask of flags/boxes
  uint8_t currentSet; // ID of current active settings/config
}__attribute__((packed)) MspStatus;

typedef struct _MspAttitude
{
  int16_t angX;       // Range [-1800,1800], Units: 1/10th degrees
  int16_t angY;       // Range [-1800,1800], Units: 1/10th degrees
  int16_t heading;    // Range [-180, 180], Units: degrees
}__attribute__((packed)) MspAttitude;

typedef struct _MspRc
{
  uint16_t roll;      // Range [1000,2000]
  uint16_t pitch;     // Range [1000,2000]
  uint16_t yaw;       // Range [1000,2000]
  uint16_t throttle;  // Range [1000,2000]
}__attribute__((packed)) MspRc;

typedef struct _MspApiVersion
{
  uint8_t protocolVersion;
  uint8_t apiVersion[2];
}__attribute__((packed)) MspApiVersion;

typedef struct _MspFcVariant
{
  char variant[4];
}__attribute__((packed)) MspFcVariant;

typedef struct _MspFcVersion
{
  uint8_t version[3];
}__attribute__((packed)) MspFcVersion;

typedef struct _MspBoardInfo
{
  char board_info[4];
  uint8_t board_version[2];
}__attribute__((packed)) MspBoardInfo;

typedef struct _MspBuildInfo
{
  char date[11];
  char time[8];
}__attribute__((packed)) MspBuildInfo;

typedef struct _MspUid
{
  uint32_t uid[3];
}__attribute__((packed)) MspUid;

typedef struct _MspSet4WayIf
{
  uint8_t connectedEscs;
}__attribute__((packed)) MspSet4WayIf;

typedef struct _MspFeatures
{
  uint32_t featureBits;
}__attribute__((packed)) MspFeatures;

typedef struct _MspMotors
{
  uint16_t data[NBR_OF_MOTORS];
}__attribute__((packed)) MspMotors;

typedef struct _MspBatteryState
{
  uint8_t cellCount;
  uint16_t capacity; // mAh
  uint8_t voltage;   // V
  uint16_t drawn;    // mAh
  uint16_t amps;     // A
  // '1.41.0',
  uint8_t state;
  uint16_t voltage2;
}__attribute__((packed)) MspBatteryState;

typedef struct _MspSetMotors
{
  uint16_t speed[8];
}__attribute__((packed)) MspSetMotors;

static bool hasSet4WayIf = false;
static paramVarId_t motorPowerSetEnableParam;
static paramVarId_t motorParams[NBR_OF_MOTORS];

// Helpers
static uint8_t mspComputeCrc(const MspHeader* header, const uint8_t* data, const uint16_t dataLen);
static bool mspIsRequestValid(MspObject* pMspObject);
static void mspProcessRequest(MspObject* pMspObject);
static void mspMakeTxPacket(MspObject* pMspObject, const msp_command_t command, const uint8_t* data, uint8_t dataLen);
static uint16_t mapMotorRequestSpeed(const uint16_t from);

// Request handlers
static void mspHandleRequestMspStatus(MspObject* pMspObject);
static void mspHandleRequestMspRc(MspObject* pMspObject);
static void mspHandleRequestMspAttitude(MspObject* pMspObject);
static void mspHandleRequestMspBoxIds(MspObject* pMspObject);
static void mspHandleRequestsApiVersion(MspObject* pMspObject);
static void mspHandleRequestsFcVariant(MspObject* pMspObject);
static void mspHandleRequestsFcVersion(MspObject* pMspObject);
static void mspHandleRequestsBoardInfo(MspObject* pMspObject);
static void mspHandleRequestsBuildInfo(MspObject* pMspObject);
static void mspHandleRequestsUiid(MspObject* pMspObject);
static void mspHandleRequestsSet4WayIf(MspObject* pMspObject);
static void mspHandleRequestMotor(MspObject* pMspObject);
static void mspHandleRequestFeaturesConfig(MspObject* pMspObject);
static void mspHandleRequestBatteryState(MspObject* pMspObject);
static void mspHandleRequestSetMotor(MspObject* pMspObject);

// Public API
void mspInit(MspObject* pMspObject, const MspResponseCallback callback)
{
  pMspObject->requestState = MSP_REQUEST_STATE_WAIT_FOR_START;
  pMspObject->responseCallback = callback;
  // Get params from internal parameter API, which we need to enable and set the motor PWM.
  motorPowerSetEnableParam = paramGetVarId("motorPowerSet", "enable");
  motorParams[0] = paramGetVarId("motorPowerSet", "m1");
  motorParams[1] = paramGetVarId("motorPowerSet", "m2");
  motorParams[2] = paramGetVarId("motorPowerSet", "m3");
  motorParams[3] = paramGetVarId("motorPowerSet", "m4");
}

void mspProcessByte(MspObject* pMspObject, const uint8_t data)
{
  // If the start token is received when we're in idle state,
  // we transition to the first state
  if((pMspObject->requestState == MSP_REQUEST_STATE_WAIT_FOR_START) &&
     (data == MSP_PREAMBLE_0))
  {
    pMspObject->requestState = MSP_REQUEST_STATE_PREAMBLE;
    pMspObject->requestHeader.preamble[0] = data;
    return;
  }

  switch(pMspObject->requestState)
  {
  case MSP_REQUEST_STATE_WAIT_FOR_START:
    // Do nothing until the start token comes
    break;

  case MSP_REQUEST_STATE_PREAMBLE:
    pMspObject->requestHeader.preamble[1] = data;
    pMspObject->requestState = MSP_REQUEST_STATE_DIRECTION;
    break;

  case MSP_REQUEST_STATE_DIRECTION:
    pMspObject->requestHeader.direction = data;
    pMspObject->requestState = MSP_REQUEST_STATE_SIZE;
    break;

  case MSP_REQUEST_STATE_SIZE:
    pMspObject->requestHeader.size = data;
    pMspObject->requestState = MSP_REQUEST_STATE_COMMAND;
    break;

  case MSP_REQUEST_STATE_COMMAND:
    pMspObject->requestHeader.command = data;

    if (pMspObject->requestHeader.size > 0)
    {
      pMspObject->dataRead = 0;
      pMspObject->requestState = MSP_REQUEST_STATE_DATA;
    }
    else
    {
      pMspObject->requestState = MSP_REQUEST_STATE_CRC;
    }
    break;

  case MSP_REQUEST_STATE_DATA:
    pMspObject->data[pMspObject->dataRead] = data;
    pMspObject->dataRead++;
    if (pMspObject->dataRead >= pMspObject->requestHeader.size)
    {
      pMspObject->requestState = MSP_REQUEST_STATE_CRC;
    }
    break;

  case MSP_REQUEST_STATE_CRC:
    pMspObject->requestCrc = data;

    // Have a completed request
    mspProcessRequest(pMspObject);

    //pMspObject->requestState = MSP_REQUEST_STATE_WAIT_FOR_START;
    break;
  }
}

bool mspHasSet4WayIf()
{
  return hasSet4WayIf;
}

void mspResetSet4WayIf()
{
  hasSet4WayIf = false;
}

// Private
static uint8_t mspComputeCrc(const MspHeader* header, const uint8_t* data, const uint16_t dataLen)
{
  // The MSP CRC is defined as the XOR of size, command,
  // and all data bytes into a zeroed sum.
  uint8_t crc = header->size ^ header->command;

  for(uint16_t i = 0; i < dataLen; i++)
  {
    crc ^= data[i];
  }

  return crc;
}

static bool mspIsRequestValid(MspObject* pMspObject)
{
  if(pMspObject->requestHeader.preamble[0] != MSP_PREAMBLE_0 ||
      pMspObject->requestHeader.preamble[1] != MSP_PREAMBLE_1)
  {
    // Invalid preamble
    DEBUG_PRINT("MSP Request has invalid preamble %d %d\n", pMspObject->requestHeader.preamble[0], pMspObject->requestHeader.preamble[1]);
    return false;
  }

  if(pMspObject->requestHeader.direction != MSP_DIRECTION_IN)
  {
    // Requests should be inputs
    DEBUG_PRINT("MSP Request has invalid direction %d\n", pMspObject->requestHeader.direction);
    return false;
  }

  if(pMspObject->requestCrc != mspComputeCrc(&pMspObject->requestHeader, pMspObject->data, pMspObject->requestHeader.size))
  {
    // CRC does not match
    DEBUG_PRINT("MSP Request has invalid crc (%d != %d)\n", pMspObject->requestCrc, mspComputeCrc(&pMspObject->requestHeader, pMspObject->data, pMspObject->requestHeader.size));
    return false;
  }

  return true;
}

static void mspProcessRequest(MspObject* pMspObject)
{
  if(!mspIsRequestValid(pMspObject))
  {
    DEBUG_PRINT("Received invalid or improperly formatted MSP request\n");
    return;
  }

  DEBUG_PRINT("Request: %d\n", pMspObject->requestHeader.command);

  switch(pMspObject->requestHeader.command)
  {
    case MSP_STATUS:
      mspHandleRequestMspStatus(pMspObject);
      break;
    case MSP_RC:
      mspHandleRequestMspRc(pMspObject);
      break;
    case MSP_ATTITUDE:
      mspHandleRequestMspAttitude(pMspObject);
      break;
    case MSP_BOXIDS:
      mspHandleRequestMspBoxIds(pMspObject);
      break;
    case MSP_API_VERSION:
      hasSet4WayIf = false;
      mspHandleRequestsApiVersion(pMspObject);
      break;
    case MSP_FC_VARIANT:
      mspHandleRequestsFcVariant(pMspObject);
      break;
    case MSP_FC_VERSION:
      mspHandleRequestsFcVersion(pMspObject);
      break;
    case MSP_BOARD_INFO:
      mspHandleRequestsBoardInfo(pMspObject);
      break;
    case MSP_BUILD_INFO:
      mspHandleRequestsBuildInfo(pMspObject);
      break;
    case MSP_UID:
      mspHandleRequestsUiid(pMspObject);
      break;
    case MSP_SET_4WAY_IF:
      mspHandleRequestsSet4WayIf(pMspObject);
      hasSet4WayIf = true;
      break;
    case MSP_MOTOR:
      mspHandleRequestMotor(pMspObject);
      break;
    case MSP_FEATURE_CONFIG:
      mspHandleRequestFeaturesConfig(pMspObject);
      break;
    case MSP_BATTERY_STATE:
      mspHandleRequestBatteryState(pMspObject);
      break;
    case MSP_SET_MOTOR:
      mspHandleRequestSetMotor(pMspObject);
      break;
    default:
      DEBUG_PRINT("Received unsupported MSP request: %d\n", pMspObject->requestHeader.command);
      return;
  }

  if(pMspObject->responseCallback)
  {
    pMspObject->responseCallback(pMspObject->mspResponse, pMspObject->mspResponseSize);
  }

  // Once we've responded, we'll go back to idle state, waiting for start.
  pMspObject->requestState = MSP_REQUEST_STATE_WAIT_FOR_START;
}

static void mspHandleRequestMspStatus(MspObject* pMspObject)
{
  MspStatus* pData = (MspStatus*)(pMspObject->mspResponse + sizeof(MspHeader));
  pData->cycleTime = 1000; // TODO: API to query this?
  pData->i2cErrors = 0; // unused
  pData->sensors = 0x0001; // no sensors supported yet, but need to report at least one to get the level bars to show
  pData->flags = 0x00000001; // always report armed (bit zero)
  pData->currentSet = 0x00;

  mspMakeTxPacket(pMspObject, MSP_STATUS, (uint8_t*) pData, sizeof(MspStatus));
}

static void mspHandleRequestMspRc(MspObject* pMspObject)
{
  MspRc* pData = (MspRc*)(pMspObject->mspResponse + sizeof(MspHeader));
  // TODO: get actual data - for now hardcode the midpoint
  pData->roll = 1500;
  pData->pitch = 1500;
  pData->yaw = 1500;
  pData->throttle = 1500;

  mspMakeTxPacket(pMspObject, MSP_RC, (uint8_t*) pData, sizeof(MspRc));
}

static void mspHandleRequestMspAttitude(MspObject* pMspObject)
{
  MspAttitude* pData = (MspAttitude*)(pMspObject->mspResponse + sizeof(MspHeader));
  float roll;
  float pitch;
  float yaw;
  sensfusion6GetEulerRPY(&roll, &pitch, &yaw); // TODO: handle kalman estimator

  pData->angX = (int16_t)(roll * 10);
  pData->angY = (int16_t)(pitch * 10);
  pData->heading = 0; // TODO: mag support

  mspMakeTxPacket(pMspObject, MSP_ATTITUDE, (uint8_t*) pData, sizeof(MspAttitude));
}

static void mspHandleRequestMspBoxIds(MspObject* pMspObject)
{
  uint8_t* pData = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader));
  // TODO: Data - this needs to be properly implemented
  // For now, we just return byte 0 = 0 which tells
  // the client to use box ID 0 for the ARMED box
  pData[0] = 0x00;

  mspMakeTxPacket(pMspObject, MSP_BOXIDS, (uint8_t*) pData, 0);
}

// Note: All request-handlers below have been reverese-engineered from BLHeli Configurator: https://github.com/stylesuxx/esc-configurator
// and ESC Configurator: https://github.com/blheli-configurator/blheli-configurator
// since there seems to be no good and valid specification for MSP extensions.

static void mspHandleRequestsApiVersion(MspObject* pMspObject)
{
  // TODO: Not sure what version we're really using... Most of the protocol was simply reverse engineered from BLHeli Configurator.
  MspApiVersion* apiVersion = (MspApiVersion*)(pMspObject->mspResponse + sizeof(MspHeader));
  apiVersion->protocolVersion = 2;
  apiVersion->apiVersion[0] = 3;
  mspMakeTxPacket(pMspObject, MSP_API_VERSION, (uint8_t*) apiVersion, sizeof(MspApiVersion));
}

static void mspHandleRequestsFcVariant(MspObject* pMspObject)
{
  MspFcVariant* fcVariant = (MspFcVariant*)(pMspObject->mspResponse + sizeof(MspHeader));
  memcpy(fcVariant->variant, "CF2 ", 4);
  mspMakeTxPacket(pMspObject, MSP_FC_VARIANT, (uint8_t*) fcVariant, sizeof(MspFcVariant));
}

static void mspHandleRequestsFcVersion(MspObject* pMspObject)
{
  MspFcVersion* fcVersion = (MspFcVersion*)(pMspObject->mspResponse + sizeof(MspHeader));
  fcVersion->version[0] = 2;
  fcVersion->version[1] = 1;
  fcVersion->version[2] = 0;
  mspMakeTxPacket(pMspObject, MSP_FC_VERSION, (uint8_t*) fcVersion, sizeof(MspFcVersion));
}

static void mspHandleRequestsBoardInfo(MspObject* pMspObject)
{
  MspBoardInfo* boardInfo = (MspBoardInfo*)(pMspObject->mspResponse + sizeof(MspHeader));
  memcpy(boardInfo->board_info, platformConfigGetDeviceTypeName(), 4);
  boardInfo->board_version[0] = 2;
  boardInfo->board_version[1] = 1;
  mspMakeTxPacket(pMspObject, MSP_BOARD_INFO, (uint8_t*) boardInfo, sizeof(MspBoardInfo));
}

static void mspHandleRequestsBuildInfo(MspObject* pMspObject)
{
  MspBuildInfo* buildInfo = (MspBuildInfo*)(pMspObject->mspResponse + sizeof(MspHeader));
  memcpy(buildInfo->date, V_STAG, 11);
  memset(buildInfo->time, 0, 8);
  mspMakeTxPacket(pMspObject, MSP_BUILD_INFO, (uint8_t*) buildInfo, sizeof(MspBuildInfo));
}

static void mspHandleRequestsUiid(MspObject* pMspObject)
{
  MspUid* uuid = (MspUid*)(pMspObject->mspResponse + sizeof(MspHeader));
  uuid->uid[0] = *((int*)(MCU_ID_ADDRESS+8));
  uuid->uid[1] = *((int*)(MCU_ID_ADDRESS+4));
  uuid->uid[2] = *((int*)(MCU_ID_ADDRESS+0));
  mspMakeTxPacket(pMspObject, MSP_UID, (uint8_t*) uuid, sizeof(MspUid));
}

static void mspHandleRequestsSet4WayIf(MspObject* pMspObject)
{
  MspSet4WayIf* set4WayIf = (MspSet4WayIf*)(pMspObject->mspResponse + sizeof(MspHeader));
  set4WayIf->connectedEscs = NBR_OF_MOTORS;
  mspMakeTxPacket(pMspObject, MSP_SET_4WAY_IF, (uint8_t*) set4WayIf, sizeof(MspSet4WayIf));
}

static void mspHandleRequestMotor(MspObject* pMspObject)
{
  MspMotors* motorData = (MspMotors*)(pMspObject->mspResponse + sizeof(MspHeader));
  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
    motorData->data[i] = 1000;
  }
  mspMakeTxPacket(pMspObject, MSP_MOTOR, (uint8_t*) motorData, sizeof(MspMotors));
}

static void mspHandleRequestFeaturesConfig(MspObject* pMspObject)
{
  MspFeatures* features = (MspFeatures*)(pMspObject->mspResponse + sizeof(MspHeader));
  features->featureBits = 0;
  mspMakeTxPacket(pMspObject, MSP_FEATURE_CONFIG, (uint8_t*) features, sizeof(MspFeatures));
}

static void mspHandleRequestBatteryState(MspObject* pMspObject)
{
  MspBatteryState* batteryState = (MspBatteryState*)(pMspObject->mspResponse + sizeof(MspHeader));
  batteryState->cellCount = 1;
  batteryState->capacity = 350;
  batteryState->voltage = 34;
  batteryState->drawn = 0;
  batteryState->amps = 0;

  batteryState->state = 0;
  batteryState->voltage2 = 0;
  mspMakeTxPacket(pMspObject, MSP_BATTERY_STATE, (uint8_t*) batteryState, sizeof(MspBatteryState));
}

static void mspHandleRequestSetMotor(MspObject* pMspObject)
{
  // Ensure that the motorPowerSet functionality is first enabled
  paramSetInt(motorPowerSetEnableParam, 1);

  for (int motor = 0; motor < NBR_OF_MOTORS; motor++) {
    // Set the motor power level for each motor.
    uint16_t motorSpeed = ((uint16_t*) pMspObject->data)[motor];
    motorSpeed = mapMotorRequestSpeed(motorSpeed);
    paramSetInt(motorParams[motor], motorSpeed);
  }

  mspMakeTxPacket(pMspObject, MSP_SET_MOTOR, 0, 0);
}

/*
 * Maps the motorSpeed from a value between 1000 and 2000 to
 * a value between 0 and 65535.
*/
static uint16_t mapMotorRequestSpeed(const uint16_t from)
{
  float perc = (from - 1000) / 1000.0;
  return perc * 65535;
}

static void mspMakeTxPacket(MspObject* pMspObject, const msp_command_t command, const uint8_t* data, uint8_t dataLen) {
  // Packet structure: http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
  MspHeader* pHeader = (MspHeader*)pMspObject->mspResponse;
  uint8_t* pData = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader));
  uint8_t* pCrc = (uint8_t*)(pMspObject->mspResponse + sizeof(MspHeader) + dataLen);

  pHeader->preamble[0] = MSP_PREAMBLE_0;
  pHeader->preamble[1] = MSP_PREAMBLE_1;
  pHeader->direction = MSP_DIRECTION_OUT;
  pHeader->size = dataLen;
  pHeader->command = command;
  memcpy(pData, data, dataLen);

  *pCrc = mspComputeCrc(pHeader, pData, pHeader->size);

  // Update the packets entire size. +1 is the CRC
  pMspObject->mspResponseSize = sizeof(MspHeader) + dataLen + 1;
}
