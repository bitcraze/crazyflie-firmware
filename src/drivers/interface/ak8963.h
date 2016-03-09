// I2Cdev library collection - AK8975 I2C device class header file
// Based on AKM AK8975/B datasheet, 12/2009
// 8/27/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-08-27 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Adapted to Crazyflie FW by Bitcraze

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _AK8963_H_
#define _AK8963_H_

#include "i2cdev.h"
#include <stdint.h>

#define AK8963_ADDRESS_00         0x0C
#define AK8963_ADDRESS_01         0x0D
#define AK8963_ADDRESS_10         0x0E // default for InvenSense MPU-6050 evaluation board
#define AK8963_ADDRESS_11         0x0F
#define AK8963_DEFAULT_ADDRESS    AK8963_ADDRESS_00

#define AK8963_RA_WIA             0x00
#define AK8963_RA_INFO            0x01
#define AK8963_RA_ST1             0x02
#define AK8963_RA_HXL             0x03
#define AK8963_RA_HXH             0x04
#define AK8963_RA_HYL             0x05
#define AK8963_RA_HYH             0x06
#define AK8963_RA_HZL             0x07
#define AK8963_RA_HZH             0x08
#define AK8963_RA_ST2             0x09
#define AK8963_RA_CNTL            0x0A
#define AK8963_RA_RSV             0x0B // RESERVED, DO NOT USE
#define AK8963_RA_ASTC            0x0C
#define AK8963_RA_TS1             0x0D // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_TS2             0x0E // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_I2CDIS          0x0F
#define AK8963_RA_ASAX            0x10
#define AK8963_RA_ASAY            0x11
#define AK8963_RA_ASAZ            0x12

#define AK8963_ST1_DRDY_BIT       0

#define AK8963_ST2_HOFL_BIT       3
#define AK8963_ST2_DERR_BIT       2

#define AK8963_CNTL_MODE_BIT      3
#define AK8963_CNTL_MODE_LENGTH   4

#define AK8963_MODE_POWERDOWN     0x00
#define AK8963_MODE_SINGLE        0x01
#define AK8963_MODE_CONT1         0x02
#define AK8963_MODE_CONT2         0x06
#define AK8963_MODE_EXTTRIG       0x04
#define AK8963_MODE_SELFTEST      0x08
#define AK8963_MODE_FUSEROM       0x0F
#define AK8963_MODE_14BIT         0x00
#define AK8963_MODE_16BIT         0x10

#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS_BIT         0

#define AK8963_ST_X_MIN           (int16_t)(-200)
#define AK8963_ST_X_MAX           (int16_t)(200)
#define AK8963_ST_Y_MIN           (int16_t)(-200)
#define AK8963_ST_Y_MAX           (int16_t)(200)
#define AK8963_ST_Z_MIN           (int16_t)(-3200)
#define AK8963_ST_Z_MAX           (int16_t)(-800)


void ak8963Init(I2C_Dev *i2cPort);
bool ak8963TestConnection();
bool ak8963SelfTest();

// WIA register
uint8_t ak8963GetDeviceID();

// INFO register
uint8_t ak8963GetInfo();

// ST1 register
bool ak8963GetDataReady();

// H* registers
void ak8963GetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t ak8963GetHeadingX();
int16_t ak8963GetHeadingY();
int16_t ak8963GetHeadingZ();

// ST2 register
bool ak8963GetOverflowStatus();
bool ak8963GetDataError();

// CNTL register
uint8_t ak8963GetMode();
void ak8963SetMode(uint8_t mode);
void ak8963Reset();

// ASTC register
void ak8963SetSelfTest(bool enabled);

// I2CDIS
void ak8963DisableI2C(); // um, why...?

// ASA* registers
void ak8963GetAdjustment(int8_t *x, int8_t *y, int8_t *z);
void ak8963SetAdjustment(int8_t x, int8_t y, int8_t z);
uint8_t ak8963GetAdjustmentX();
void ak8963SetAdjustmentX(uint8_t x);
uint8_t ak8963GetAdjustmentY();
void ak8963SetAdjustmentY(uint8_t y);
uint8_t ak8963GetAdjustmentZ();
void ak8963SetAdjustmentZ(uint8_t z);

#endif /* _AK8963_H_ */
