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
/* DW1000 registers */

#ifndef __DW1000_H__
#define __DW1000_H__

// Time resolution in micro-seconds of time based registers/values.
// Each bit in a timestamp counts for a period of approx. 15.65ps
#define TIME_RES 0.000015650040064103
#define TIME_RES_INV 63897.6

// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
#define DISTANCE_OF_RADIO 0.0046917639786159f
#define DISTANCE_OF_RADIO_INV 213.139451293f

// time stamp byte length
#define LEN_STAMP 5

// enum to determine RX or TX mode of device
#define IDLE_MODE 0x00
#define RX_MODE 0x01
#define TX_MODE 0x02

// used for SPI ready w/o actual writes
#define JUNK 0x00

// no sub-address for register write
#define NO_SUB 0x00

// device id register
#define DEV_ID 0x00
#define LEN_DEV_ID 4

// extended unique identifier register
#define EUI 0x01
#define LEN_EUI 8

// PAN identifier, short address register
#define PANADR 0x03
#define LEN_PANADR 4

// device configuration register
#define SYS_CFG 0x04
#define LEN_SYS_CFG 4
#define FFEN_BIT 0
#define FFBC_BIT 1
#define FFAB_BIT 2
#define FFAD_BIT 3
#define FFAA_BIT 4
#define FFAM_BIT 5
#define FFAR_BIT 6
#define DIS_DRXB_BIT 12
#define DIS_STXP_BIT 18
#define HIRQ_POL_BIT 9
#define RXAUTR_BIT 29
#define PHR_MODE_SUB 16
#define LEN_PHR_MODE_SUB 2
#define RXM110K_BIT 22
#define RXWTOE_BIT 28

// device control register
#define SYS_CTRL 0x0D
#define LEN_SYS_CTRL 4
#define SFCST_BIT 0
#define TXSTRT_BIT 1
#define TXDLYS_BIT 2
#define TRXOFF_BIT 6
#define WAIT4RESP_BIT 7
#define RXENAB_BIT 8
#define RXDLYS_BIT 9

// system event status register
#define SYS_STATUS 0x0F
#define LEN_SYS_STATUS 5
#define CPLOCK_BIT 1
#define AAT_BIT 3
#define TXFRB_BIT 4
#define TXPRS_BIT 5
#define TXPHS_BIT 6
#define TXFRS_BIT 7
#define LDEDONE_BIT 10
#define RXPHE_BIT 12
#define RXDFR_BIT 13
#define RXFCG_BIT 14
#define RXFCE_BIT 15
#define RXRFSL_BIT 16
#define RXRFTO_BIT 17
#define LDEERR_BIT 18
#define RFPLL_LL_BIT 24
#define CLKPLL_LL_BIT 25

// GPIO control register
#define GPIO_CTRL 0x26
#define GPIO_MODE_SUB 0x00
#define LEN_GPIO_MODE 4
#define GPIO_DIR_SUB 0x08
#define LEN_GPIO_DIR 4


// system event mask register
// NOTE: uses the bit definitions of SYS_STATUS (below 32)
#define SYS_MASK 0x0E
#define LEN_SYS_MASK 4

// system time counter
#define SYS_TIME 0x06
#define LEN_SYS_TIME LEN_STAMP

// RX timestamp register
#define RX_TIME 0x15
#define LEN_RX_TIME 14
#define RX_STAMP_SUB 0x00
#define FP_AMPL1_SUB 0x07
#define LEN_RX_STAMP LEN_STAMP
#define LEN_FP_AMPL1 2

// RX frame quality
#define RX_FQUAL 0x12
#define LEN_RX_FQUAL 8
#define STD_NOISE_SUB 0x00
#define FP_AMPL2_SUB 0x02
#define FP_AMPL3_SUB 0x04
#define CIR_PWR_SUB 0x06
#define LEN_STD_NOISE 2
#define LEN_FP_AMPL2 2
#define LEN_FP_AMPL3 2
#define LEN_CIR_PWR 2

// TX timestamp register
#define TX_TIME 0x17
#define LEN_TX_TIME 10
#define TX_STAMP_SUB 0
#define LEN_TX_STAMP LEN_STAMP

// timing register (for delayed RX/TX)
#define DX_TIME 0x0A
#define LEN_DX_TIME LEN_STAMP

// transmit data buffer
#define TX_BUFFER 0x09
#define LEN_TX_BUFFER 1024
#define LEN_UWB_FRAMES 127
#define LEN_EXT_UWB_FRAMES 1023

// Receive frame ait timeout period
#define RX_FWTO 0x0C

// RX frame info
#define RX_FINFO 0x10
#define LEN_RX_FINFO 4

// receive data buffer
#define RX_BUFFER 0x11
#define LEN_RX_BUFFER 1024

// transmit control
#define TX_FCTRL 0x08
#define LEN_TX_FCTRL 5

// channel control
#define CHAN_CTRL 0x1F
#define LEN_CHAN_CTRL 4
#define DWSFD_BIT 17
#define TNSSFD_BIT 20
#define RNSSFD_BIT 21

// user-defined SFD
#define USR_SFD 0x21
#define LEN_USR_SFD 41
#define SFD_LENGTH_SUB 0x00
#define LEN_SFD_LENGTH 1

// OTP control (for LDE micro code loading only)
#define OTP_IF 0x2D
#define OTP_ADDR_SUB 0x04
#define OTP_CTRL_SUB 0x06
#define OTP_RDAT_SUB 0x0A
#define LEN_OTP_ADDR 2
#define LEN_OTP_CTRL 2
#define LEN_OTP_RDAT 4

// AGC_TUNE1/2 (for re-tuning only)
#define AGC_TUNE 0x23
#define AGC_TUNE1_SUB 0x04
#define AGC_TUNE2_SUB 0x0C
#define AGC_TUNE3_SUB 0x12
#define LEN_AGC_TUNE1 2
#define LEN_AGC_TUNE2 4
#define LEN_AGC_TUNE3 2

// DRX_TUNE2 (for re-tuning only)
#define DRX_TUNE 0x27
#define DRX_TUNE0b_SUB 0x02
#define DRX_TUNE1a_SUB 0x04
#define DRX_TUNE1b_SUB 0x06
#define DRX_TUNE2_SUB 0x08
#define DRX_TUNE4H_SUB 0x26
#define LEN_DRX_TUNE0b 2
#define LEN_DRX_TUNE1a 2
#define LEN_DRX_TUNE1b 2
#define LEN_DRX_TUNE2 4
#define LEN_DRX_TUNE4H 2

// LDE_CFG1 (for re-tuning only)
#define LDE_IF 0x2E
#define LDE_CFG1_SUB 0x0806
#define LDE_RXANTD_SUB 0x1804
#define LDE_CFG2_SUB 0x1806
#define LDE_REPC_SUB 0x2804
#define LEN_LDE_CFG1 1
#define LEN_LDE_CFG2 2
#define LEN_LDE_REPC 2
#define LEN_LDE_RXANTD 2

// TX_POWER (for re-tuning only)
#define TX_POWER 0x1E
#define LEN_TX_POWER 4

// RF_CONF (for re-tuning only)
#define RF_CONF 0x28
#define RF_RXCTRLH_SUB 0x0B
#define RF_TXCTRL_SUB 0x0C
#define LEN_RF_RXCTRLH 1
#define LEN_RF_TXCTRL 4

// TX_CAL (for re-tuning only)
#define TX_CAL 0x2A
#define TC_PGDELAY_SUB 0x0B
#define LEN_TC_PGDELAY 1

// FS_CTRL (for re-tuning only)
#define FS_CTRL 0x2B
#define FS_PLLCFG_SUB 0x07
#define FS_PLLTUNE_SUB 0x0B
#define FS_XTALT_SUB 0x0E
#define LEN_FS_PLLCFG 4
#define LEN_FS_PLLTUNE 1
#define LEN_FS_XTALT 1

// PMSC
#define PMSC 0x36
#define PMSC_CTRL0_SUB 0x00
#define LEN_PMSC_CTRL0 4
#define PMSC_LEDC 0x28
#define LEN_PMSC_LEDC 4

// TX_ANTD Antenna delays
#define TX_ANTD 0x18
#define LEN_TX_ANTD 2

/* Settings */
// transmission/reception bit rate
#define TRX_RATE_110KBPS 0x00
#define TRX_RATE_850KBPS 0x01
#define TRX_RATE_6800KBPS 0x02

// transmission pulse frequency
// 0x00 is 4MHZ, but receiver in DW1000 does not support it (!??)
#define TX_PULSE_FREQ_16MHZ 0x01
#define TX_PULSE_FREQ_64MHZ 0x02

// preamble length (PE + TXPSR bits)
#define TX_PREAMBLE_LEN_64 0x01
#define TX_PREAMBLE_LEN_128 0x05
#define TX_PREAMBLE_LEN_256 0x09
#define TX_PREAMBLE_LEN_512 0x0D
#define TX_PREAMBLE_LEN_1024 0x02
#define TX_PREAMBLE_LEN_1536 0x06
#define TX_PREAMBLE_LEN_2048 0x0A
#define TX_PREAMBLE_LEN_4096 0x03

// PAC size. */
#define PAC_SIZE_8 8
#define PAC_SIZE_16 16
#define PAC_SIZE_32 32
#define PAC_SIZE_64 64

/* channel of operation. */
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define CHANNEL_4 4
#define CHANNEL_5 5
#define CHANNEL_7 7

/* preamble codes. */
#define PREAMBLE_CODE_16MHZ_1 1
#define PREAMBLE_CODE_16MHZ_2 2
#define PREAMBLE_CODE_16MHZ_3 3
#define PREAMBLE_CODE_16MHZ_4 4
#define PREAMBLE_CODE_16MHZ_5 5
#define PREAMBLE_CODE_16MHZ_6 6
#define PREAMBLE_CODE_16MHZ_7 7
#define PREAMBLE_CODE_16MHZ_8 8
#define PREAMBLE_CODE_64MHZ_9 9
#define PREAMBLE_CODE_64MHZ_10 10
#define PREAMBLE_CODE_64MHZ_11 11
#define PREAMBLE_CODE_64MHZ_12 12
#define PREAMBLE_CODE_64MHZ_17 17
#define PREAMBLE_CODE_64MHZ_18 18
#define PREAMBLE_CODE_64MHZ_19 19
#define PREAMBLE_CODE_64MHZ_20 20

/* frame length settings. */
#define FRAME_LENGTH_NORMAL 0x00
#define FRAME_LENGTH_EXTENDED 0x03

#endif // __DW1000_H__
