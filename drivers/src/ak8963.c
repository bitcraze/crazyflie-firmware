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
#define DEBUG_MODULE "AK8963"

#include "FreeRTOS.h"
#include "task.h"

#include "ak8963.h"

// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"

static uint8_t devAddr;
static uint8_t buffer[6];
static I2C_Dev *I2Cx;
static bool isInit;

static bool ak8963EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char* string);

/** Power on and prepare for general usage.
 * No specific pre-configuration is necessary for this device.
 */
void ak8963Init(I2C_Dev *i2cPort)
{
  if (isInit)
    return;

  I2Cx = i2cPort;
  devAddr = AK8963_ADDRESS_00;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ak8963TestConnection()
{
  if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_WIA, buffer) == 1)
  {
    return (buffer[0] == 0x48);
  }
  return false;
}

bool ak8963SelfTest()
{
  bool testStatus = true;
  int16_t mx, my, mz;  // positive magnetometer measurements
  uint8_t confSave;
  uint8_t timeout = 20;

  // Save register values
  if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_CNTL, &confSave) == false)
  {
    // TODO: error handling
    return false;
  }

  // Power down
  ak8963SetMode(AK8963_MODE_POWERDOWN);
  ak8963SetSelfTest(true);
  ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_SELFTEST);
  // Clear ST1 by reading ST2
  ak8963GetOverflowStatus();
  while (!ak8963GetDataReady() && timeout--)
  {
    vTaskDelay(M2T(1));
  }
  ak8963GetHeading(&mx, &my, &mz);
  // Power down
  ak8963SetMode(AK8963_MODE_POWERDOWN);

  if (ak8963EvaluateSelfTest(AK8963_ST_X_MIN, AK8963_ST_X_MAX, mx, "X") &&
      ak8963EvaluateSelfTest(AK8963_ST_Y_MIN, AK8963_ST_Y_MAX, my, "Y") &&
      ak8963EvaluateSelfTest(AK8963_ST_Z_MIN, AK8963_ST_Z_MAX, mz, "Z"))
   {
    DEBUG_PRINT("Self test [OK].\n");
  }
  else
  {
    testStatus = false;
  }

  // Power up with saved config
  ak8963SetMode(confSave);

  return testStatus;
}

/** Evaluate the values from a HMC8335L self test.
 * @param min The min limit of the self test
 * @param max The max limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within min - max limit, false otherwise
 */
static bool ak8963EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char* string)
{
  if (value < min || value > max)
  {
    DEBUG_PRINT("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n",
                string, min, max, value);
    return false;
  }
  return true;
}

// WIA register

uint8_t ak8963GetDeviceID()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_WIA, buffer);
  return buffer[0];
}

// INFO register

uint8_t ak8963GetInfo()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_INFO, buffer);
  return buffer[0];
}

// ST1 register

bool ak8963GetDataReady()
{
  i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, buffer);
  return buffer[0];
}

// H* registers
void ak8963GetHeading(int16_t *x, int16_t *y, int16_t *z)
{
//  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
//  delay(10);
  i2cdevRead(I2Cx, devAddr, AK8963_RA_HXL, 6, buffer);
  *x = (((int16_t) buffer[1]) << 8) | buffer[0];
  *y = (((int16_t) buffer[3]) << 8) | buffer[2];
  *z = (((int16_t) buffer[5]) << 8) | buffer[4];
}
int16_t ak8963GetHeadingX()
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
//  delay(10);
  i2cdevRead(I2Cx, devAddr, AK8963_RA_HXL, 2, buffer);
  return (((int16_t) buffer[1]) << 8) | buffer[0];
}
int16_t ak8963GetHeadingY()
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
//  delay(10);
  i2cdevRead(I2Cx, devAddr, AK8963_RA_HYL, 2, buffer);
  return (((int16_t) buffer[1]) << 8) | buffer[0];
}
int16_t ak8963GetHeadingZ()
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_SINGLE);
//  delay(10);
  i2cdevRead(I2Cx, devAddr, AK8963_RA_HZL, 2, buffer);
  return (((int16_t) buffer[1]) << 8) | buffer[0];
}

// ST2 register
bool ak8963GetOverflowStatus()
{
  i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, buffer);
  return buffer[0];
}
bool ak8963GetDataError()
{
  i2cdevReadBit(I2Cx, devAddr, AK8963_RA_ST2, AK8963_ST2_DERR_BIT, buffer);
  return buffer[0];
}

// CNTL register
uint8_t ak8963GetMode()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_CNTL, buffer);
  return buffer[0];
}
void ak8963SetMode(uint8_t mode)
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, mode);
}
void ak8963Reset()
{
  i2cdevWriteBits(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_CNTL_MODE_BIT,
      AK8963_CNTL_MODE_LENGTH, AK8963_MODE_POWERDOWN);
}

// ASTC register
void ak8963SetSelfTest(bool enabled)
{
  i2cdevWriteBit(I2Cx, devAddr, AK8963_RA_ASTC, AK8963_ASTC_SELF_BIT, enabled);
}

// I2CDIS
void ak8963DisableI2C()
{
  i2cdevWriteBit(I2Cx, devAddr, AK8963_RA_I2CDIS, AK8963_I2CDIS_BIT, true);
}

// ASA* registers
void ak8963GetAdjustment(int8_t *x, int8_t *y, int8_t *z)
{
  i2cdevRead(I2Cx, devAddr, AK8963_RA_ASAX, 3, buffer);
  *x = buffer[0];
  *y = buffer[1];
  *z = buffer[2];
}
void ak8963SetAdjustment(int8_t x, int8_t y, int8_t z)
{
  buffer[0] = x;
  buffer[1] = y;
  buffer[2] = z;
  i2cdevWrite(I2Cx, devAddr, AK8963_RA_ASAX, 3, buffer);
}
uint8_t ak8963GetAdjustmentX()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAX, buffer);
  return buffer[0];
}
void ak8963SetAdjustmentX(uint8_t x)
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAX, x);
}
uint8_t ak8963GetAdjustmentY()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAY, buffer);
  return buffer[0];
}
void ak8963SetAdjustmentY(uint8_t y)
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAY, y);
}
uint8_t ak8963GetAdjustmentZ()
{
  i2cdevReadByte(I2Cx, devAddr, AK8963_RA_ASAZ, buffer);
  return buffer[0];
}
void ak8963SetAdjustmentZ(uint8_t z)
{
  i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_ASAZ, z);
}
