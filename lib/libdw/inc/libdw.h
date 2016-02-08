/*
 * Driver for decaWave DW1000 802.15.4 UWB radio chip.
 *
 * Copyright (c) 2016 Bitcraze AB
 * Converted to C from  the Decawave DW1000 library for arduino.
 * which is Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LIBDW_H__
#define __LIBDW_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "dw1000.h"

struct dwOps_s;
struct dwDevice_s;

typedef union dwTime_u {
  uint8_t raw[5];
  uint64_t full;
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} dwTime_t;

typedef void (*dwHandler_t)(struct dwDevice_s *dev);

/**
 * DW device type. Contains the context of a dw1000 device and should be passed
 * as first argument of most of the driver functions.
 */
typedef struct dwDevice_s {
  struct dwOps_s *ops;
  void *userdata;

  /* State */
  uint8_t sysctrl[LEN_SYS_CTRL];
  uint8_t deviceMode;
  uint8_t networkAndAddress[LEN_PANADR];
  uint8_t syscfg[LEN_SYS_CFG];
  uint8_t sysmask[LEN_SYS_MASK];
  uint8_t chanctrl[LEN_CHAN_CTRL];
  uint8_t sysstatus[LEN_SYS_STATUS];
  uint8_t txfctrl[LEN_TX_FCTRL];

  uint8_t extendedFrameLength;
  uint8_t pacSize;
  uint8_t pulseFrequency;
  uint8_t dataRate;
  uint8_t preambleLength;
  uint8_t preambleCode;
  uint8_t channel;
  bool smartPower;
  bool frameCheck;
  bool permanentReceive;
  bool wait4resp;

  dwTime_t antennaDelay;

  // Callback handles
  dwHandler_t handleSent;
  dwHandler_t handleReceived;
  dwHandler_t handleReceiveTimeout;
} dwDevice_t;

typedef enum {dwSpiSpeedLow, dwSpiSpeedHigh} dwSpiSpeed_t;

typedef enum {dwClockAuto = 0x00, dwClockXti = 0x01, dwClockPll = 0x02} dwClock_t;

// Default Mode of operation
extern const uint8_t MODE_LONGDATA_RANGE_LOWPOWER[];
extern const uint8_t MODE_SHORTDATA_FAST_LOWPOWER[];
extern const uint8_t MODE_LONGDATA_FAST_LOWPOWER[];
extern const uint8_t MODE_SHORTDATA_FAST_ACCURACY[];
extern const uint8_t MODE_LONGDATA_FAST_ACCURACY[];
extern const uint8_t MODE_LONGDATA_RANGE_ACCURACY[];

/**
 * DW operation type. Constains function pointer to all hardware-dependent
 * operation required to access the DW1000 device.
 */
typedef struct dwOps_s {
  /**
   * Function that activates the chip-select, sends header, read data and
   * disable the chip-select.
   */
  void (*spiRead)(dwDevice_t* dev, const void *header, size_t headerLength,
                                   void* data, size_t dataLength);

  /**
   * Function that activates the chip-select, sends header, sends data and
   * disable the chip-select.
   */
  void (*spiWrite)(dwDevice_t* dev, const void *header, size_t headerLength,
                                    const void* data, size_t dataLength);

  /**
   * Sets the SPI bus speed. Take as argument:
   *   - dwSpiSpeedLow: <= 4MHz
   *   - dwSpiSpeedHigh: <= 20MHz
   */
  void (*spiSetSpeed)(dwDevice_t* dev, dwSpiSpeed_t speed);

  /**
   * Waits at least 'delay' miliseconds.
   */
  void (*delayms)(dwDevice_t* dev, unsigned int delay);

  /**
   * Resets the DW1000 by pulling the reset pin low and then releasing it.
   * This function is optional, if not set softreset via SPI will be used.
   */
   void (*reset)(dwDevice_t *dev);
} dwOps_t;

/**
 * Initialize the device data structure.
 */
void dwInit(dwDevice_t* dev, dwOps_t* ops);

/**
 * Set a userData pointer to the device.
 */
void dwSetUserdata(dwDevice_t* dev, void* userdata);

/**
 * Get the userData pointer from the device.
 */
void* dwGetUserdata(dwDevice_t* dev);

/**
 * Setup the DW1000
 */
int dwConfigure(dwDevice_t* dev);

/**
 * Read and return the device ID, only chip with ID 0xdeca0130 is supported.
 */
uint32_t dwGetDeviceId(dwDevice_t* dev);

/**
 * Manualy blinks LEDs.
 * @param leds Bit-field of the LEDs to blink
 */
void dwEnableAllLeds(dwDevice_t* dev);

/**
 * Sets clock Mode
 */
void dwEnableClock(dwDevice_t* dev, dwClock_t clock);

/**
 * Resets the chip via SPI
 */
void dwSoftReset(dwDevice_t* dev);

void dwManageLDE(dwDevice_t* dev);

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

void dwReadSystemConfigurationRegister(dwDevice_t* dev);
void dwWriteSystemConfigurationRegister(dwDevice_t* dev);
void dwReadSystemEventStatusRegister(dwDevice_t* dev);
void dwReadNetworkIdAndDeviceAddress(dwDevice_t* dev);
void dwWriteNetworkIdAndDeviceAddress(dwDevice_t* dev);
void dwReadSystemEventMaskRegister(dwDevice_t* dev);
void dwWriteSystemEventMaskRegister(dwDevice_t* dev);
void dwReadChannelControlRegister(dwDevice_t* dev);
void dwWriteChannelControlRegister(dwDevice_t* dev);
void dwReadTransmitFrameControlRegister(dwDevice_t* dev);
void dwWriteTransmitFrameControlRegister(dwDevice_t* dev);

/****************************************************************/

/**
 * Set Receive Wait Timeout.
 * @param timeout Timeout in step of 1.026us (512 count of the dw1000
 *                 fundamental 499.2MHz clock) or 0 to disable the timeout.
 *
 * @note dwCommitConfiguration() should be called if this function
 * enables or disables the timeout. If the timeout is just updated and not
 * enabled this function will update to the new timeout and nothing more has to
 * be done.
 */
void dwSetReceiveWaitTimeout(dwDevice_t *dev, uint16_t timeout);

void dwSetFrameFilter(dwDevice_t* dev, bool val);
void dwSetFrameFilterBehaveCoordinator(dwDevice_t* dev, bool val);
void dwSetFrameFilterAllowBeacon(dwDevice_t* dev, bool val);
void dwSetFrameFilterAllowData(dwDevice_t* dev, bool val);
void dwSetFrameFilterAllowAcknowledgement(dwDevice_t* dev, bool val);
void dwSetFrameFilterAllowMAC(dwDevice_t* dev, bool val);
void dwSetFrameFilterAllowReserved(dwDevice_t* dev, bool val);
void dwSetDoubleBuffering(dwDevice_t* dev, bool val);
void dwSetInterruptPolarity(dwDevice_t* dev, bool val);
void dwSetReceiverAutoReenable(dwDevice_t* dev, bool val);
void dwInterruptOnSent(dwDevice_t* dev, bool val);
void dwInterruptOnReceived(dwDevice_t* dev, bool val);
void dwInterruptOnReceiveFailed(dwDevice_t* dev, bool val);
void dwInterruptOnReceiveTimeout(dwDevice_t* dev, bool val);
void dwInterruptOnReceiveTimestampAvailable(dwDevice_t* dev, bool val);
void dwInterruptOnAutomaticAcknowledgeTrigger(dwDevice_t* dev, bool val);
void dwClearInterrupts(dwDevice_t* dev);



void dwIdle(dwDevice_t* dev);
void dwNewReceive(dwDevice_t* dev);
void dwStartReceive(dwDevice_t* dev);
void dwNewTransmit(dwDevice_t* dev);
void dwStartTransmit(dwDevice_t* dev);
void dwNewConfiguration(dwDevice_t* dev);
void dwCommitConfiguration(dwDevice_t* dev);
void dwWaitForResponse(dwDevice_t* dev, bool val);
void dwSuppressFrameCheck(dwDevice_t* dev, bool val);
void dwUseSmartPower(dwDevice_t* dev, bool smartPower);
dwTime_t dwSetDelay(dwDevice_t* dev, const dwTime_t* delay);
void dwSetDataRate(dwDevice_t* dev, uint8_t rate);
void dwSetPulseFrequency(dwDevice_t* dev, uint8_t freq);
uint8_t dwGetPulseFrequency(dwDevice_t* dev);
void dwSetPreambleLength(dwDevice_t* dev, uint8_t prealen);
void dwUseExtendedFrameLength(dwDevice_t* dev, bool val);
void dwReceivePermanently(dwDevice_t* dev, bool val);
void dwSetChannel(dwDevice_t* dev, uint8_t channel);
void dwSetPreambleCode(dwDevice_t* dev, uint8_t preacode);
void dwSetDefaults(dwDevice_t* dev);
void dwSetData(dwDevice_t* dev, uint8_t data[], unsigned int n);
unsigned int dwGetDataLength(dwDevice_t* dev);
void dwGetData(dwDevice_t* dev, uint8_t data[], unsigned int n);
void dwGetTransmitTimestamp(dwDevice_t* dev, dwTime_t* time);
void dwGetReceiveTimestamp(dwDevice_t* dev, dwTime_t* time);
void dwCorrectTimestamp(dwDevice_t* dev, dwTime_t* timestamp);
void dwGetSystemTimestamp(dwDevice_t* dev, dwTime_t* time);
bool dwIsTransmitDone(dwDevice_t* dev);
bool dwIsReceiveTimestampAvailable(dwDevice_t* dev);
bool dwIsReceiveDone(dwDevice_t* dev);
bool dwIsReceiveFailed(dwDevice_t *dev);
bool dwIsReceiveTimeout(dwDevice_t* dev);
bool dwIsClockProblem(dwDevice_t* dev);
void dwClearAllStatus(dwDevice_t* dev);
void dwClearReceiveTimestampAvailableStatus(dwDevice_t* dev);
void dwClearReceiveStatus(dwDevice_t* dev);
void dwClearTransmitStatus(dwDevice_t* dev);
float dwGetReceiveQuality(dwDevice_t* dev);
float dwGetFirstPathPower(dwDevice_t* dev);
float dwGetReceivePower(dwDevice_t* dev);
void dwEnableMode(dwDevice_t *dev, const uint8_t mode[]);
void dwTune(dwDevice_t *dev);
void dwHandleInterrupt(dwDevice_t *dev);
// float getFirstPathPower(dwDevice_t *dev);
// float getReceivePower(dwDevice_t *dev);

void dwAttachSentHandler(dwDevice_t *dev, dwHandler_t handler);
void dwAttachReceivedHandler(dwDevice_t *dev, dwHandler_t handler);
void dwAttachReceiveTimeoutHandler(dwDevice_t *dev, dwHandler_t handler);

void dwSetAntenaDelay(dwDevice_t *dev, dwTime_t delay);

/* Tune the DWM radio parameters */
void dwTune(dwDevice_t *dev);



/**
 * Put the dwm1000 in idle mode
 */
void dwIdle(dwDevice_t* dev);

/**
 * Returns a human-readable error string
 */
char* dwStrError(int error);

/**
 * Read from the dw1000 SPI interface
 */
void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                void* data, size_t length);

uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address);

/**
 * Read from the dw1000 SPI interface
 */
void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                 const void* data, size_t length);

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                  uint32_t data);

/* Error codes */
#define DW_ERROR_OK 0
#define DW_ERROR_WRONG_ID 1

#endif //__LIBDW_H__
