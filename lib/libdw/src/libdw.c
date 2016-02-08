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

#include <string.h>
#include <math.h>

#include <libdw.h>
#include <dw1000.h>


static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;

// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31,   0,  36,  65,  84,  97, 106, 110, 112};
static const uint8_t BIAS_500_64[] = {110, 105, 100,  93,  82,  69,  51, 27,  0, 21,  35,  42,  49,  62,  71,  76,  81,  86};
static const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69,  47,  25,  0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29,  0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

// Default Mode of operation
const uint8_t MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const uint8_t MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};

// Useful shortcuts
#define delayms(delay) dev->ops->delayms(dev, delay)

// Utility functions
static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val);
static void writeValueToBytes(uint8_t data[], long val, unsigned int n);
static bool getBit(uint8_t data[], unsigned int n, unsigned int bit);

static void dummy(){
  ;
}

void dwInit(dwDevice_t* dev, dwOps_t* ops)
{
  dev->ops = ops;
  dev->userdata = NULL;

  /* Device default state */
  dev->extendedFrameLength = FRAME_LENGTH_NORMAL;
  dev->pacSize = PAC_SIZE_8;
  dev->pulseFrequency = TX_PULSE_FREQ_16MHZ;
  dev->dataRate = TRX_RATE_6800KBPS;
  dev->preambleLength = TX_PREAMBLE_LEN_128;
  dev->preambleCode = PREAMBLE_CODE_16MHZ_4;
  dev->channel = CHANNEL_5;
  dev->smartPower = false;
  dev->frameCheck = true;
  dev->permanentReceive = false;
  dev->deviceMode = IDLE_MODE;

  writeValueToBytes(dev->antennaDelay.raw, 16384, LEN_STAMP);

  // Dummy callback handlers
  dev->handleSent = dummy;
  dev->handleReceived = dummy;

}

void dwSetUserdata(dwDevice_t* dev, void* userdata)
{
  dev->userdata = userdata;
}

void* dwGetUserdata(dwDevice_t* dev)
{
  return dev->userdata;
}

int dwConfigure(dwDevice_t* dev)
{
  dwEnableClock(dev, dwClockAuto);
  delayms(5);

  // Reset the chip
  if (dev->ops->reset) {
    dev->ops->reset(dev);
  } else {
    dwSoftReset(dev);
  }

  if (dwGetDeviceId(dev) != 0xdeca0130) {
    return DW_ERROR_WRONG_ID;
  }

  // Set default address
  memset(dev->networkAndAddress, 0xff, LEN_PANADR);
  dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);

  // default configuration
  memset(dev->syscfg, 0, LEN_SYS_CFG);
  dwSetDoubleBuffering(dev, false);
	dwSetInterruptPolarity(dev, true);
	dwWriteSystemConfigurationRegister(dev);
	// default interrupt mask, i.e. no interrupts
	dwClearInterrupts(dev);
	dwWriteSystemEventMaskRegister(dev);
	// load LDE micro-code
	dwEnableClock(dev, dwClockXti);
	delayms(5);
	dwManageLDE(dev);
	delayms(5);
	dwEnableClock(dev, dwClockPll);
	delayms(5);
  //dev->ops->spiSetSpeed(dev, dwSpiSpeedHigh);

  // //Enable LED clock
  // dwSpiWrite32(dev, PMSC, PMSC_CTRL0_SUB, dwSpiRead32(dev, PMSC, PMSC_CTRL0_SUB) | 0x008C0000);
  //
  // // Setup all LEDs
  //
  // dwSpiWrite32(dev, 0x26, 0x00, dwSpiRead32(dev, 0x26, 0x00) | 0x1540);
  //
  // // Start the pll
  //
  // delayms(1);

  // Initialize for default configuration (as per datasheet)

  return DW_ERROR_OK;
}

void dwManageLDE(dwDevice_t* dev) {
	// transfer any ldo tune values
	// uint8_t ldoTune[LEN_OTP_RDAT];
	// readBytesOTP(0x04, ldoTune); // TODO #define
	// if(ldoTune[0] != 0) {
	// 	// TODO tuning available, copy over to RAM: use OTP_LDO bit
	// }
	// tell the chip to load the LDE microcode
	// TODO remove clock-related code (PMSC_CTRL) as handled separately
	uint8_t pmscctrl0[LEN_PMSC_CTRL0];
	uint8_t otpctrl[LEN_OTP_CTRL];
	memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
	memset(otpctrl, 0, LEN_OTP_CTRL);
	dwSpiRead(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	dwSpiRead(dev, OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
	pmscctrl0[0] = 0x01;
	pmscctrl0[1] = 0x03;
	otpctrl[0] = 0x00;
	otpctrl[1] = 0x80;
	dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	dwSpiWrite(dev, OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
	delayms(5);
	pmscctrl0[0] = 0x00;
	pmscctrl0[1] = 0x02;
	dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}


uint32_t dwGetDeviceId(dwDevice_t* dev)
{
  uint32_t devid;

  dwSpiRead(dev, DEV_ID, 0, &devid, sizeof(devid));

  return devid;
}

void dwEnableAllLeds(dwDevice_t* dev)
{
  uint32_t reg;

  // Set all 4 GPIO in LED mode
  reg = dwSpiRead32(dev, GPIO_CTRL, GPIO_MODE_SUB);
  reg &= ~0x00003FC0ul;
  reg |= 0x00001540ul;
  dwSpiWrite32(dev, GPIO_CTRL, GPIO_MODE_SUB, reg);

  // Enable debounce clock (used to clock the LED blinking)
  reg = dwSpiRead32(dev, PMSC, PMSC_CTRL0_SUB);
  reg |= 0x00840000ul;
  dwSpiWrite32(dev, PMSC, PMSC_CTRL0_SUB, reg);

  // Enable LED blinking and set the rate
  reg = 0x00000110ul;
  dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);

  // Trigger a manual blink of the LEDs for test
  reg |= 0x000f0000ul;
  dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);
  reg &= ~0x000f0000ul;
  dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);
}

void dwEnableClock(dwDevice_t* dev, dwClock_t clock) {
	uint8_t pmscctrl0[LEN_PMSC_CTRL0];
	memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
	dwSpiRead(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	if(clock == dwClockAuto) {
    dev->ops->spiSetSpeed(dev, dwSpiSpeedLow);
		pmscctrl0[0] = dwClockAuto;
		pmscctrl0[1] &= 0xFE;
	} else if(clock == dwClockXti) {
    dev->ops->spiSetSpeed(dev, dwSpiSpeedLow);
		pmscctrl0[0] &= 0xFC;
		pmscctrl0[0] |= dwClockXti;
	} else if(clock == dwClockPll) {
    dev->ops->spiSetSpeed(dev, dwSpiSpeedHigh);
		pmscctrl0[0] &= 0xFC;
		pmscctrl0[0] |= dwClockPll;
	} else {
		// TODO deliver proper warning
	}
	dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, 1);
  dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}

void dwSoftReset(dwDevice_t* dev)
{
  uint8_t pmscctrl0[LEN_PMSC_CTRL0];
  dwSpiRead(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[0] = 0x01;
  dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[3] = 0x00;
  dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  delayms(10);
  pmscctrl0[0] = 0x00;
  pmscctrl0[3] = 0xF0;
  dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  // force into idle mode
  dwIdle(dev);
}

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

void dwReadSystemConfigurationRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_CFG, NO_SUB, dev->syscfg, LEN_SYS_CFG);
}

void dwWriteSystemConfigurationRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, SYS_CFG, NO_SUB, dev->syscfg, LEN_SYS_CFG);
}

void dwReadSystemEventStatusRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

void dwReadNetworkIdAndDeviceAddress(dwDevice_t* dev) {
	dwSpiRead(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
}

void dwWriteNetworkIdAndDeviceAddress(dwDevice_t* dev) {
	dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
}

void dwReadSystemEventMaskRegister(dwDevice_t* dev) {
	dwSpiRead(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
}

void dwWriteSystemEventMaskRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
}

void dwReadChannelControlRegister(dwDevice_t* dev) {
	dwSpiRead(dev, CHAN_CTRL, NO_SUB, dev->chanctrl, LEN_CHAN_CTRL);
}

void dwWriteChannelControlRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, CHAN_CTRL, NO_SUB, dev->chanctrl, LEN_CHAN_CTRL);
}

void dwReadTransmitFrameControlRegister(dwDevice_t* dev) {
	dwSpiRead(dev, TX_FCTRL, NO_SUB, dev->txfctrl, LEN_TX_FCTRL);
}

void dwWriteTransmitFrameControlRegister(dwDevice_t* dev) {
	dwSpiWrite(dev, TX_FCTRL, NO_SUB, dev->txfctrl, LEN_TX_FCTRL);
}

/******************************************************************/

void dwSetReceiveWaitTimeout(dwDevice_t *dev, uint16_t timeout) {
  dwSpiWrite(dev, RX_FWTO, NO_SUB, &timeout, 2);
  setBit(dev->syscfg, LEN_SYS_CFG, RXWTOE_BIT, timeout!=0);
}

void dwSetFrameFilter(dwDevice_t* dev, bool val) {
	setBit(dev->syscfg, LEN_SYS_CFG, FFEN_BIT, val);
}

void dwSetFrameFilterBehaveCoordinator(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFBC_BIT, val);
}

void dwSetFrameFilterAllowBeacon(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFAB_BIT, val);
}

void dwSetFrameFilterAllowData(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFAD_BIT, val);
}

void dwSetFrameFilterAllowAcknowledgement(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFAA_BIT, val);
}

void dwSetFrameFilterAllowMAC(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFAM_BIT, val);
}

void dwSetFrameFilterAllowReserved(dwDevice_t* dev, bool val) {
    setBit(dev->syscfg, LEN_SYS_CFG, FFAR_BIT, val);
}

void dwSetDoubleBuffering(dwDevice_t* dev, bool val) {
	setBit(dev->syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}

void dwSetInterruptPolarity(dwDevice_t* dev, bool val) {
	setBit(dev->syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}

void dwSetReceiverAutoReenable(dwDevice_t* dev, bool val) {
	setBit(dev->syscfg, LEN_SYS_CFG, RXAUTR_BIT, val);
}

void dwInterruptOnSent(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_MASK, TXFRS_BIT, val);
}

void dwInterruptOnReceived(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_MASK, RXDFR_BIT, val);
	setBit(dev->sysmask, LEN_SYS_MASK, RXFCG_BIT, val);
}

void dwInterruptOnReceiveFailed(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_STATUS, LDEERR_BIT, val);
	setBit(dev->sysmask, LEN_SYS_STATUS, RXFCE_BIT, val);
	setBit(dev->sysmask, LEN_SYS_STATUS, RXPHE_BIT, val);
	setBit(dev->sysmask, LEN_SYS_STATUS, RXRFSL_BIT, val);
}

void dwInterruptOnReceiveTimeout(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_MASK, RXRFTO_BIT, val);
}

void dwInterruptOnReceiveTimestampAvailable(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_MASK, LDEDONE_BIT, val);
}

void dwInterruptOnAutomaticAcknowledgeTrigger(dwDevice_t* dev, bool val) {
	setBit(dev->sysmask, LEN_SYS_MASK, AAT_BIT, val);
}

void dwClearInterrupts(dwDevice_t* dev) {
	memset(dev->sysmask, 0, LEN_SYS_MASK);
}

void dwIdle(dwDevice_t* dev)
{
   memset(dev->sysctrl, 0, LEN_SYS_CTRL);
   dev->sysctrl[0] |= 1<<TRXOFF_BIT;
   dev->deviceMode = IDLE_MODE;
   dwSpiWrite(dev, SYS_CTRL, NO_SUB, dev->sysctrl, LEN_SYS_CTRL);
}

void dwNewReceive(dwDevice_t* dev) {
	dwIdle(dev);
	memset(dev->sysctrl, 0, LEN_SYS_CTRL);
	dwClearReceiveStatus(dev);
	dev->deviceMode = RX_MODE;
}

void dwStartReceive(dwDevice_t* dev) {
	setBit(dev->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dev->frameCheck);
	setBit(dev->sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
	dwSpiWrite(dev, SYS_CTRL, NO_SUB, dev->sysctrl, LEN_SYS_CTRL);
}

void dwNewTransmit(dwDevice_t* dev) {
	dwIdle(dev);
	memset(dev->sysctrl, 0, LEN_SYS_CTRL);
	dwClearTransmitStatus(dev);
	dev->deviceMode = TX_MODE;
}

void dwStartTransmit(dwDevice_t* dev) {
	dwWriteTransmitFrameControlRegister(dev);
	setBit(dev->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dev->frameCheck);
	setBit(dev->sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, true);
	dwSpiWrite(dev, SYS_CTRL, NO_SUB, dev->sysctrl, LEN_SYS_CTRL);
	if(dev->permanentReceive) {
		memset(dev->sysctrl, 0, LEN_SYS_CTRL);
		dev->deviceMode = RX_MODE;
		dwStartReceive(dev);
	} else if (dev->wait4resp) {
    dev->deviceMode = RX_MODE;
  } else {
		dev->deviceMode = IDLE_MODE;
	}
}

void dwNewConfiguration(dwDevice_t* dev) {
	dwIdle(dev);
	dwReadNetworkIdAndDeviceAddress(dev);
	dwReadSystemConfigurationRegister(dev);
	dwReadChannelControlRegister(dev);
	dwReadTransmitFrameControlRegister(dev);
	dwReadSystemEventMaskRegister(dev);
}

void dwCommitConfiguration(dwDevice_t* dev) {
	// write all configurations back to device
	dwWriteNetworkIdAndDeviceAddress(dev);
	dwWriteSystemConfigurationRegister(dev);
	dwWriteChannelControlRegister(dev);
	dwWriteTransmitFrameControlRegister(dev);
	dwWriteSystemEventMaskRegister(dev);
	// tune according to configuration
	dwTune(dev);
	// TODO clean up code + antenna delay/calibration API
	// TODO setter + check not larger two bytes integer
	// uint8_t antennaDelayBytes[LEN_STAMP];
	// writeValueToBytes(antennaDelayBytes, 16384, LEN_STAMP);
	// dev->antennaDelay.setTimestamp(antennaDelayBytes);
	// dwSpiRead(dev, TX_ANTD, NO_SUB, antennaDelayBytes, LEN_TX_ANTD);
  // dwSpiRead(dev, LDE_IF, LDE_RXANTD_SUB, antennaDelayBytes, LEN_LDE_RXANTD);
  dwSpiWrite(dev, TX_ANTD, NO_SUB, dev->antennaDelay.raw, LEN_TX_ANTD);
  dwSpiWrite(dev, LDE_IF, LDE_RXANTD_SUB, dev->antennaDelay.raw, LEN_LDE_RXANTD);
}

void dwWaitForResponse(dwDevice_t* dev, bool val) {
  dev->wait4resp = val;
	setBit(dev->sysctrl, LEN_SYS_CTRL, WAIT4RESP_BIT, val);
}

void dwSuppressFrameCheck(dwDevice_t* dev, bool val) {
	dev->frameCheck = !val;
}

void dwUseSmartPower(dwDevice_t* dev, bool smartPower) {
  dev->smartPower = smartPower;
	setBit(dev->syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);
}

dwTime_t dwSetDelay(dwDevice_t* dev, const dwTime_t* delay) {
	if(dev->deviceMode == TX_MODE) {
		setBit(dev->sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
	} else if(dev->deviceMode == RX_MODE) {
		setBit(dev->sysctrl, LEN_SYS_CTRL, RXDLYS_BIT, true);
	} else {
		// in idle, ignore
    dwTime_t zero = {.full = 0};
		return zero;
	}
	uint8_t delayBytes[5];
	dwTime_t futureTime;
	dwGetSystemTimestamp(dev, &futureTime);
	futureTime.full += delay->full;
  memcpy(delayBytes, futureTime.raw, sizeof(futureTime.raw));
	delayBytes[0] = 0;
	delayBytes[1] &= 0xFE;
	dwSpiWrite(dev, DX_TIME, NO_SUB, delayBytes, LEN_DX_TIME);
	// adjust expected time with configured antenna delay
  memcpy(futureTime.raw, delayBytes, sizeof(futureTime.raw));
	futureTime.full += dev->antennaDelay.full;
	return futureTime;
}



void dwSetDataRate(dwDevice_t* dev, uint8_t rate) {
	rate &= 0x03;
	dev->txfctrl[1] &= 0x83;
	dev->txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);
	// special 110kbps flag
	if(rate == TRX_RATE_110KBPS) {
		setBit(dev->syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
	} else {
		setBit(dev->syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
	}
	// SFD mode and type (non-configurable, as in Table )
	if(rate == TRX_RATE_6800KBPS) {
		setBit(dev->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
		setBit(dev->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
		setBit(dev->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
	} else {
		setBit(dev->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
		setBit(dev->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
		setBit(dev->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);

	}
	uint8_t sfdLength;
	if(rate == TRX_RATE_6800KBPS) {
		sfdLength = 0x08;
	} else if(rate == TRX_RATE_850KBPS) {
		sfdLength = 0x10;
	} else {
		sfdLength = 0x40;
	}
	dwSpiRead(dev, USR_SFD, SFD_LENGTH_SUB, &sfdLength, LEN_SFD_LENGTH);
	dev->dataRate = rate;
}

void dwSetPulseFrequency(dwDevice_t* dev, uint8_t freq) {
	freq &= 0x03;
	dev->txfctrl[2] &= 0xFC;
	dev->txfctrl[2] |= (uint8_t)(freq & 0xFF);
	dev->chanctrl[2] &= 0xF3;
	dev->chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
	dev->pulseFrequency = freq;

}

uint8_t dwGetPulseFrequency(dwDevice_t* dev) {
    return dev->pulseFrequency;
}

void dwSetPreambleLength(dwDevice_t* dev, uint8_t prealen) {
	prealen &= 0x0F;
	dev->txfctrl[2] &= 0xC3;
	dev->txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);
	if(prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
		dev->pacSize = PAC_SIZE_8;
	} else if(prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
		dev->pacSize = PAC_SIZE_16;
	} else if(prealen == TX_PREAMBLE_LEN_1024) {
		dev->pacSize = PAC_SIZE_32;
	} else {
		dev->pacSize = PAC_SIZE_64;
	}
	dev->preambleLength = prealen;
}

void dwUseExtendedFrameLength(dwDevice_t* dev, bool val) {
	dev->extendedFrameLength = (val ? FRAME_LENGTH_EXTENDED : FRAME_LENGTH_NORMAL);
	dev->syscfg[2] &= 0xFC;
	dev->syscfg[2] |= dev->extendedFrameLength;
}

void dwReceivePermanently(dwDevice_t* dev, bool val) {
	dev->permanentReceive = val;
	if(val) {
		// in case permanent, also reenable receiver once failed
		dwSetReceiverAutoReenable(dev, true);
		dwWriteSystemConfigurationRegister(dev);
	}
}

void dwSetChannel(dwDevice_t* dev, uint8_t channel) {
	channel &= 0xF;
	dev->chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
	dev->channel = channel;
}

void dwSetPreambleCode(dwDevice_t* dev, uint8_t preacode) {
	preacode &= 0x1F;
	dev->chanctrl[2] &= 0x3F;
	dev->chanctrl[2] |= ((preacode << 6) & 0xFF);
	dev->chanctrl[3] = 0x00;
	dev->chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
	dev->preambleCode = preacode;
}

void dwSetDefaults(dwDevice_t* dev) {
	if(dev->deviceMode == TX_MODE) {

	} else if(dev->deviceMode == RX_MODE) {

	} else if(dev->deviceMode == IDLE_MODE) {
		dwUseExtendedFrameLength(dev, false);
		dwUseSmartPower(dev, false);
		dwSuppressFrameCheck(dev, false);
    //for global frame filtering
		dwSetFrameFilter(dev, false);
    //for data frame (poll, poll_ack, range, range report, range failed) filtering
    dwSetFrameFilterAllowData(dev, false);
    //for reserved (blink) frame filtering
    dwSetFrameFilterAllowReserved(dev, false);
    //setFrameFilterAllowMAC(true);
    //setFrameFilterAllowBeacon(true);
    //setFrameFilterAllowAcknowledgement(true);
		dwInterruptOnSent(dev, true);
		dwInterruptOnReceived(dev, true);
    dwInterruptOnReceiveTimeout(dev, true);
		dwInterruptOnReceiveFailed(dev, false);
		dwInterruptOnReceiveTimestampAvailable(dev, false);
		dwInterruptOnAutomaticAcknowledgeTrigger(dev, false);
		dwSetReceiverAutoReenable(dev, true);
		// default mode when powering up the chip
		// still explicitly selected for later tuning
		dwEnableMode(dev, MODE_LONGDATA_RANGE_LOWPOWER);
	}
}

void dwSetData(dwDevice_t* dev, uint8_t data[], unsigned int n) {
	if(dev->frameCheck) {
		n+=2; // two bytes CRC-16
	}
	if(n > LEN_EXT_UWB_FRAMES) {
		return; // TODO proper error handling: frame/buffer size
	}
	if(n > LEN_UWB_FRAMES && !dev->extendedFrameLength) {
		return; // TODO proper error handling: frame/buffer size
	}
	// transmit data and length
	dwSpiWrite(dev, TX_BUFFER, NO_SUB, data, n);
	dev->txfctrl[0] = (uint8_t)(n & 0xFF); // 1 byte (regular length + 1 bit)
	dev->txfctrl[1] &= 0xE0;
	dev->txfctrl[1] |= (uint8_t)((n >> 8) & 0x03);	// 2 added bits if extended length
}

unsigned int dwGetDataLength(dwDevice_t* dev) {
	unsigned int len = 0;
	if(dev->deviceMode == TX_MODE) {
		// 10 bits of TX frame control register
		len = ((((unsigned int)dev->txfctrl[1] << 8) | (unsigned int)dev->txfctrl[0]) & 0x03FF);
	} else if(dev->deviceMode == RX_MODE) {
		// 10 bits of RX frame control register
		uint8_t rxFrameInfo[LEN_RX_FINFO];
		dwSpiRead(dev, RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
		len = ((((unsigned int)rxFrameInfo[1] << 8) | (unsigned int)rxFrameInfo[0]) & 0x03FF);
	}
	if(dev->frameCheck && len > 2) {
		return len-2;
	}
	return len;
}

void dwGetData(dwDevice_t* dev, uint8_t data[], unsigned int n) {
	if(n <= 0) {
		return;
	}
	dwSpiRead(dev, RX_BUFFER, NO_SUB, data, n);
}

void dwGetTransmitTimestamp(dwDevice_t* dev, dwTime_t* time) {
	dwSpiRead(dev, TX_TIME, TX_STAMP_SUB, time->raw, LEN_TX_STAMP);
}

void dwGetReceiveTimestamp(dwDevice_t* dev, dwTime_t* time) {
  time->full = 0;
	dwSpiRead(dev, RX_TIME, RX_STAMP_SUB, time->raw, LEN_RX_STAMP);
	// correct timestamp (i.e. consider range bias)
	dwCorrectTimestamp(dev, time);
}

void dwCorrectTimestamp(dwDevice_t* dev, dwTime_t* timestamp) {
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase = -(dwGetReceivePower(dev) + 61.0f) * 0.5f;
	if (!isfinite(rxPowerBase)) {
	  return;
	}
	int rxPowerBaseLow = (int)rxPowerBase;
	int rxPowerBaseHigh = rxPowerBaseLow + 1;
	if(rxPowerBaseLow < 0) {
		rxPowerBaseLow = 0;
		rxPowerBaseHigh = 0;
	} else if(rxPowerBaseHigh > 17) {
		rxPowerBaseLow = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int rangeBiasHigh = 0;
	int rangeBiasLow = 0;
	if(dev->channel == CHANNEL_4 || dev->channel == CHANNEL_7) {
		// 900 MHz receiver bandwidth
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else {
			// TODO proper error handling
		}
	} else {
		// 500 MHz receiver bandwidth
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		} else {
			// TODO proper error handling
		}
	}
	// linear interpolation of bias values
	float rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	dwTime_t adjustmentTime;
  adjustmentTime.full = (int)(rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
	// apply correction
	timestamp->full += adjustmentTime.full;
}

void dwGetSystemTimestamp(dwDevice_t* dev, dwTime_t* time) {
	dwSpiRead(dev, SYS_TIME, NO_SUB, time->raw, LEN_SYS_TIME);
}

bool dwIsTransmitDone(dwDevice_t* dev) {
	return getBit(dev->sysstatus, LEN_SYS_STATUS, TXFRS_BIT);
}

bool dwIsReceiveTimestampAvailable(dwDevice_t* dev) {
	return getBit(dev->sysstatus, LEN_SYS_STATUS, LDEDONE_BIT);
}

bool dwIsReceiveDone(dwDevice_t* dev) {
	if(dev->frameCheck) {
		return getBit(dev->sysstatus, LEN_SYS_STATUS, RXFCG_BIT);
	}
	return getBit(dev->sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
}

bool dwIsReceiveFailed(dwDevice_t *dev) {
	bool ldeErr, rxCRCErr, rxHeaderErr, rxDecodeErr;
	ldeErr = getBit(dev->sysstatus, LEN_SYS_STATUS, LDEERR_BIT);
	rxCRCErr = getBit(dev->sysstatus, LEN_SYS_STATUS, RXFCE_BIT);
	rxHeaderErr = getBit(dev->sysstatus, LEN_SYS_STATUS, RXPHE_BIT);
	rxDecodeErr = getBit(dev->sysstatus, LEN_SYS_STATUS, RXRFSL_BIT);
	if(ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr) {
		return true;
	}
	return false;
}

bool dwIsReceiveTimeout(dwDevice_t* dev) {
	return getBit(dev->sysstatus, LEN_SYS_STATUS, RXRFTO_BIT);
}

bool dwIsClockProblem(dwDevice_t* dev) {
	bool clkllErr, rfllErr;
	clkllErr = getBit(dev->sysstatus, LEN_SYS_STATUS, CLKPLL_LL_BIT);
	rfllErr = getBit(dev->sysstatus, LEN_SYS_STATUS, RFPLL_LL_BIT);
	if(clkllErr || rfllErr) {
		return true;
	}
	return false;
}

void dwClearAllStatus(dwDevice_t* dev) {
	memset(dev->sysstatus, 0, LEN_SYS_STATUS);
	dwSpiWrite(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

void dwClearReceiveTimestampAvailableStatus(dwDevice_t* dev) {
	setBit(dev->sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
	dwSpiWrite(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

void dwClearReceiveStatus(dwDevice_t* dev) {
	// clear latched RX bits (i.e. write 1 to clear)
	setBit(dev->sysstatus, LEN_SYS_STATUS, RXDFR_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, LDEERR_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, RXPHE_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, RXFCE_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, RXFCG_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, RXRFSL_BIT, true);
	dwSpiWrite(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

void dwClearTransmitStatus(dwDevice_t* dev) {
	// clear latched TX bits
	setBit(dev->sysstatus, LEN_SYS_STATUS, TXFRB_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, TXPRS_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, TXPHS_BIT, true);
	setBit(dev->sysstatus, LEN_SYS_STATUS, TXFRS_BIT, true);
	dwSpiWrite(dev, SYS_STATUS, NO_SUB, dev->sysstatus, LEN_SYS_STATUS);
}

float dwGetReceiveQuality(dwDevice_t* dev) {
	uint8_t noiseBytes[LEN_STD_NOISE];
	uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
	unsigned int noise, f2;
	dwSpiRead(dev, RX_FQUAL, STD_NOISE_SUB, noiseBytes, LEN_STD_NOISE);
	dwSpiRead(dev, RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	noise = (unsigned int)noiseBytes[0] | ((unsigned int)noiseBytes[1] << 8);
	f2 = (unsigned int)fpAmpl2Bytes[0] | ((unsigned int)fpAmpl2Bytes[1] << 8);
	return (float)f2 / noise;
}

float dwGetFirstPathPower(dwDevice_t* dev) {
	uint8_t fpAmpl1Bytes[LEN_FP_AMPL1];
	uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
	uint8_t fpAmpl3Bytes[LEN_FP_AMPL3];
	uint8_t rxFrameInfo[LEN_RX_FINFO];
	unsigned int f1, f2, f3, N;
	float A, corrFac;
	dwSpiRead(dev, RX_TIME, FP_AMPL1_SUB, fpAmpl1Bytes, LEN_FP_AMPL1);
	dwSpiRead(dev, RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	dwSpiRead(dev, RX_FQUAL, FP_AMPL3_SUB, fpAmpl3Bytes, LEN_FP_AMPL3);
	dwSpiRead(dev, RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	f1 = (unsigned int)fpAmpl1Bytes[0] | ((unsigned int)fpAmpl1Bytes[1] << 8);
	f2 = (unsigned int)fpAmpl2Bytes[0] | ((unsigned int)fpAmpl2Bytes[1] << 8);
	f3 = (unsigned int)fpAmpl3Bytes[0] | ((unsigned int)fpAmpl3Bytes[1] << 8);
	N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A = 115.72;
		corrFac = 2.3334;
	} else {
		A = 121.74;
		corrFac = 1.1667;
	}
	float estFpPwr = 10.0 * log10(((float)f1 * (float)f1 + (float)f2 * (float)f2 + (float)f3 * (float)f3) / ((float)N * (float)N)) - A;
	if(estFpPwr <= -88) {
		return estFpPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr + 88) * corrFac;
	}
	return estFpPwr;
}

float dwGetReceivePower(dwDevice_t* dev) {
	uint8_t cirPwrBytes[LEN_CIR_PWR];
	uint8_t rxFrameInfo[LEN_RX_FINFO];
	unsigned long twoPower17 = 131072;
	unsigned int C, N;
	float A, corrFac;
	dwSpiRead(dev, RX_FQUAL, CIR_PWR_SUB, cirPwrBytes, LEN_CIR_PWR);
	dwSpiRead(dev, RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	C = (unsigned int)cirPwrBytes[0] | ((unsigned int)cirPwrBytes[1] << 8);
	N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A = 115.72;
		corrFac = 2.3334;
	} else {
		A = 121.74;
		corrFac = 1.1667;
	}
	float estRxPwr = 10.0 * log10(((float)C * (float)twoPower17) / ((float)N * (float)N)) - A;
	if(estRxPwr <= -88) {
		return estRxPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estRxPwr += (estRxPwr + 88) * corrFac;
	}
	return estRxPwr;
}

void dwEnableMode(dwDevice_t *dev, const uint8_t mode[]) {
	dwSetDataRate(dev, mode[0]);
	dwSetPulseFrequency(dev, mode[1]);
	dwSetPreambleLength(dev, mode[2]);
	// TODO add channel and code to mode tuples
	// TODO add channel and code settings with checks (see Table 58)
	dwSetChannel(dev, CHANNEL_5);
	if(mode[1] == TX_PULSE_FREQ_16MHZ) {
		dwSetPreambleCode(dev, PREAMBLE_CODE_16MHZ_4);
	} else {
		dwSetPreambleCode(dev, PREAMBLE_CODE_64MHZ_10);
	}
}

void dwTune(dwDevice_t *dev) {
	// these registers are going to be tuned/configured
	uint8_t agctune1[LEN_AGC_TUNE1];
	uint8_t agctune2[LEN_AGC_TUNE2];
	uint8_t agctune3[LEN_AGC_TUNE3];
	uint8_t drxtune0b[LEN_DRX_TUNE0b];
	uint8_t drxtune1a[LEN_DRX_TUNE1a];
	uint8_t drxtune1b[LEN_DRX_TUNE1b];
	uint8_t drxtune2[LEN_DRX_TUNE2];
	uint8_t drxtune4H[LEN_DRX_TUNE4H];
	uint8_t ldecfg1[LEN_LDE_CFG1];
	uint8_t ldecfg2[LEN_LDE_CFG2];
	uint8_t lderepc[LEN_LDE_REPC];
	uint8_t txpower[LEN_TX_POWER];
	uint8_t rfrxctrlh[LEN_RF_RXCTRLH];
	uint8_t rftxctrl[LEN_RF_TXCTRL];
	uint8_t tcpgdelay[LEN_TC_PGDELAY];
	uint8_t fspllcfg[LEN_FS_PLLCFG];
	uint8_t fsplltune[LEN_FS_PLLTUNE];
	// uint8_t fsxtalt[LEN_FS_XTALT];
	// AGC_TUNE1
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
	} else {
		// TODO proper error/warning handling
	}
	// AGC_TUNE2
	writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
	// AGC_TUNE3
	writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
	// DRX_TUNE0b (already optimized according to Table 20 of user manual)
	if(dev->dataRate == TRX_RATE_110KBPS) {
		writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_850KBPS) {
		writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_6800KBPS) {
		writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1a
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1b
	if(dev->preambleLength ==  TX_PREAMBLE_LEN_1536 || dev->preambleLength ==  TX_PREAMBLE_LEN_2048 ||
			dev->preambleLength ==  TX_PREAMBLE_LEN_4096) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->preambleLength != TX_PREAMBLE_LEN_64) {
		if(dev->dataRate == TRX_RATE_850KBPS || dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		if(dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	}
	// DRX_TUNE2
	if(dev->pacSize == PAC_SIZE_8) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_16) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_32) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_64) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE4H
	if(dev->preambleLength == TX_PREAMBLE_LEN_64) {
		writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
	} else {
		writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
	}
	// RF_RXCTRLH
	if(dev->channel != CHANNEL_4 && dev->channel != CHANNEL_7) {
		writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
	} else {
		writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
	}
	// RX_TXCTRL
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
	} else {
		// TODO proper error/warning handling
	}
	// TC_PGDELAY
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
	} else {
		// TODO proper error/warning handling
	}
	// FS_PLLCFG and FS_PLLTUNE
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_2 || dev->channel == CHANNEL_4) {
		writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x5E, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_5 || dev->channel == CHANNEL_7) {
		writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0xA6, LEN_FS_PLLTUNE);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_CFG1
	writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
	// LDE_CFG2
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_REPC
	if(dev->preambleCode == PREAMBLE_CODE_16MHZ_1 || dev->preambleCode == PREAMBLE_CODE_16MHZ_2) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_3 || dev->preambleCode == PREAMBLE_CODE_16MHZ_8) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_4) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_5) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_6) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_7) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_9) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_10 || dev->preambleCode == PREAMBLE_CODE_64MHZ_17) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_11) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_12) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_18 || dev->preambleCode == PREAMBLE_CODE_64MHZ_19) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_20) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
		}
	} else {
		// TODO proper error/warning handling
	}
	// TX_POWER (enabled smart transmit power control)
	if(dev->channel == CHANNEL_1 || dev->channel == CHANNEL_2) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_3) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_4) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_5) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_7) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// mid range XTAL trim (TODO here we assume no calibration data available in OTP)
	//writeValueToBytes(fsxtalt, 0x60, LEN_FS_XTALT);
	// write configuration back to chip
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
	dwSpiWrite(dev, LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
	dwSpiWrite(dev, LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
	dwSpiWrite(dev, LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
	dwSpiWrite(dev, TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
	dwSpiWrite(dev, RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
	dwSpiWrite(dev, RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
	dwSpiWrite(dev, TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
	dwSpiWrite(dev, FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
	dwSpiWrite(dev, FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
	//dwSpiWrite(dev, FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}

// FIXME: This is a test!
void (*_handleError)(void) = dummy;
void (*_handleReceiveFailed)(void) = dummy;
void (*_handleReceiveTimestampAvailable)(void) = dummy;

void dwHandleInterrupt(dwDevice_t *dev) {
	// read current status and handle via callbacks
	dwReadSystemEventStatusRegister(dev);
	if(dwIsClockProblem(dev) /* TODO and others */ && _handleError != 0) {
		(*_handleError)();
	}
	if(dwIsTransmitDone(dev) && dev->handleSent != 0) {
		(*dev->handleSent)(dev);
		dwClearTransmitStatus(dev);
	}
	if(dwIsReceiveTimestampAvailable(dev) && _handleReceiveTimestampAvailable != 0) {
		(*_handleReceiveTimestampAvailable)();
		dwClearReceiveTimestampAvailableStatus(dev);
	}
	if(dwIsReceiveFailed(dev) && _handleReceiveFailed != 0) {
		(*_handleReceiveFailed)();
		dwClearReceiveStatus(dev);
		if(dev->permanentReceive) {
			dwNewReceive(dev);
			dwStartReceive(dev);
		}
	} else if(dwIsReceiveTimeout(dev) && dev->handleReceiveTimeout != 0) {
		(*dev->handleReceiveTimeout)(dev);
		dwClearReceiveStatus(dev);
		if(dev->permanentReceive) {
			dwNewReceive(dev);
			dwStartReceive(dev);
		}
	} else if(dwIsReceiveDone(dev) && dev->handleReceived != 0) {
		(*dev->handleReceived)(dev);
		dwClearReceiveStatus(dev);
		if(dev->permanentReceive) {
			dwNewReceive(dev);
			dwStartReceive(dev);
		}
	}
	// clear all status that is left unhandled
	dwClearAllStatus(dev);
}

void dwAttachSentHandler(dwDevice_t *dev, dwHandler_t handler)
{
  dev->handleSent = handler;
}

void dwAttachReceivedHandler(dwDevice_t *dev, dwHandler_t handler)
{
  dev->handleReceived = handler;
}

void dwAttachReceiveTimeoutHandler(dwDevice_t *dev, dwHandler_t handler) {
  dev->handleReceiveTimeout = handler;
}

float getFirstPathPower(dwDevice_t *dev) {
	uint8_t fpAmpl1Bytes[LEN_FP_AMPL1];
	uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
	uint8_t fpAmpl3Bytes[LEN_FP_AMPL3];
	uint8_t rxFrameInfo[LEN_RX_FINFO];
	unsigned int f1, f2, f3, N;
	float A, corrFac;
	dwSpiRead(dev, RX_TIME, FP_AMPL1_SUB, fpAmpl1Bytes, LEN_FP_AMPL1);
	dwSpiRead(dev, RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	dwSpiRead(dev, RX_FQUAL, FP_AMPL3_SUB, fpAmpl3Bytes, LEN_FP_AMPL3);
	dwSpiRead(dev, RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	f1 = (unsigned int)fpAmpl1Bytes[0] | ((unsigned int)fpAmpl1Bytes[1] << 8);
	f2 = (unsigned int)fpAmpl2Bytes[0] | ((unsigned int)fpAmpl2Bytes[1] << 8);
	f3 = (unsigned int)fpAmpl3Bytes[0] | ((unsigned int)fpAmpl3Bytes[1] << 8);
	N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A = 115.72;
		corrFac = 2.3334;
	} else {
		A = 121.74;
		corrFac = 1.1667;
	}
	float estFpPwr = 10.0 * log10(((float)f1 * (float)f1 + (float)f2 * (float)f2 + (float)f3 * (float)f3) / ((float)N * (float)N)) - A;
	if(estFpPwr <= -88) {
		return estFpPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr + 88) * corrFac;
	}
	return estFpPwr;
}

float getReceivePower(dwDevice_t *dev) {
	uint8_t cirPwrBytes[LEN_CIR_PWR];
	uint8_t rxFrameInfo[LEN_RX_FINFO];
	unsigned long twoPower17 = 131072;
	unsigned int C, N;
	float A, corrFac;
	dwSpiRead(dev, RX_FQUAL, CIR_PWR_SUB, cirPwrBytes, LEN_CIR_PWR);
	dwSpiRead(dev, RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	C = (unsigned int)cirPwrBytes[0] | ((unsigned int)cirPwrBytes[1] << 8);
	N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A = 115.72;
		corrFac = 2.3334;
	} else {
		A = 121.74;
		corrFac = 1.1667;
	}
	float estRxPwr = 10.0 * log10(((float)C * (float)twoPower17) / ((float)N * (float)N)) - A;
	if(estRxPwr <= -88) {
		return estRxPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estRxPwr += (estRxPwr + 88) * corrFac;
	}
	return estRxPwr;
}

void dwSetAntenaDelay(dwDevice_t *dev, dwTime_t delay) {
  dev->antennaDelay.full = delay.full;
}

void dwSpiRead(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                void* data, size_t length)
{
  uint8_t header[3];
  size_t headerLength=1;

  header[0] = regid & 0x3f;

  if (address != 0) {
    header[0] |= 0x40;

    header[1] = address & 0x7f;
    address >>= 7;
    headerLength = 2;

    if (address != 0) {
      header[1] |= 0x80;
      header[2] = address & 0xff;
      headerLength = 3;
    }
  }

  dev->ops->spiRead(dev, header, headerLength, data, length);
}

uint32_t dwSpiRead32(dwDevice_t *dev, uint8_t regid, uint32_t address)
{
  uint32_t data;
  dwSpiRead(dev, regid, address, &data, sizeof(data));
  return data;
}

void dwSpiWrite(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                 const void* data, size_t length)
{
  uint8_t header[3];
  size_t headerLength=1;

  header[0] = regid & 0x3f;
  header[0] |= 0x80;

  if (address != 0) {
    header[0] |= 0x40;

    header[1] = address & 0x7f;
    address >>= 7;
    headerLength = 2;

    if (address != 0) {
      header[1] |= 0x80;
      header[2] = address & 0xff;
      headerLength = 3;
    }
  }

  dev->ops->spiWrite(dev, header, headerLength, data, length);
}

void dwSpiWrite32(dwDevice_t *dev, uint8_t regid, uint32_t address,
                                   uint32_t data)
{
  dwSpiWrite(dev, regid, address, &data, sizeof(data));
}

char* dwStrError(int error)
{
  if (error == DW_ERROR_OK) return "No error";
  else if (error == DW_ERROR_WRONG_ID) return "Wrong chip ID";
  else return "Uknown error";
}

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
	int idx;
	int shift;

	idx = bit / 8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	uint8_t* targetByte = &data[idx];
	shift = bit % 8;
	if(val) {
		*targetByte |= (1<<shift);
	} else {
	  *targetByte &= ~(1<<shift);
	}
}

static bool getBit(uint8_t data[], unsigned int n, unsigned int bit) {
	int idx;
	int shift;

	idx = bit / 8;
	if(idx >= n) {
		return false; // TODO proper error handling: out of bounds
	}
	uint8_t targetByte = data[idx];
	shift = bit % 8;

	return (targetByte>>shift)&0x01;
}

static void writeValueToBytes(uint8_t data[], long val, unsigned int n) {
	int i;
	for(i = 0; i < n; i++) {
		data[i] = ((val >> (i * 8)) & 0xFF);
	}
}
