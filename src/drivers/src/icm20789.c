// I2Cdev library collection - MPU6500 I2C device class
// Based on InvenSense MPU-6500 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

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
#define DEBUG_MODULE "ICM20789"

#include <math.h>
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nvicconf.h"

// TA: Maybe not so good to bring in these dependencies...
#include "debug.h"
#include "eprintf.h"

#include "mpu6500.h"
#include "icm20789.h"

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RZR_GPIO_CS             GPIO_Pin_14
#define RZR_GPIO_CS_PORT        GPIOC
#define RZR_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOC

#define RZR_SPI                 SPI2
#define RZR_SPI_AF              GPIO_AF_SPI2
#define RZR_SPI_CLK             RCC_APB1Periph_SPI2
#define RZR_GPIO_SPI_PORT       GPIOB
#define RZR_GPIO_SPI_CLK        RCC_AHB1Periph_GPIOB
#define RZR_GPIO_SPI_SCK        GPIO_Pin_13
#define RZR_GPIO_SPI_SCK_SRC    GPIO_PinSource13
#define RZR_GPIO_SPI_MISO       GPIO_Pin_14
#define RZR_GPIO_SPI_MISO_SRC   GPIO_PinSource14
#define RZR_GPIO_SPI_MOSI       GPIO_Pin_15
#define RZR_GPIO_SPI_MOSI_SRC   GPIO_PinSource15

#define RZR_SPI_DMA_IRQ_PRIO        NVIC_HIGH_PRI
#define RZR_SPI_DMA                 DMA1
#define RZR_SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define RZR_SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define RZR_SPI_RX_DMA_STREAM       DMA1_Stream3
#define RZR_SPI_RX_DMA_IRQ          DMA1_Stream3_IRQn
#define RZR_SPI_RX_DMA_IRQHandler   DMA1_Stream3_IRQHandler
#define RZR_SPI_RX_DMA_CHANNEL      DMA_Channel_0
#define RZR_SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define RZR_SPI_TX_DMA_STREAM       DMA1_Stream4
#define RZR_SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define RZR_SPI_TX_DMA_IRQHandler   DMA1_Stream4_IRQHandler
#define RZR_SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define RZR_SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define DUMMY_BYTE    0x00
/* Usefull macro */
#define RZR_EN_CS() GPIO_ResetBits(RZR_GPIO_CS_PORT, RZR_GPIO_CS)
#define RZR_DIS_CS() GPIO_SetBits(RZR_GPIO_CS_PORT, RZR_GPIO_CS)

/* Defines and buffers for full duplex SPI DMA transactions */
#define SENSORS_READ_START_CMD      (MPU6500_RA_ACCEL_XOUT_H | 0x80)
#define SENSORS_READ_TRANSACTION_SIZE 15 // 1 byte command followed by 14 bytes data
static uint8_t spiTxBuffer[SENSORS_READ_TRANSACTION_SIZE];
static uint8_t spiRxBuffer[SENSORS_READ_TRANSACTION_SIZE];
static xSemaphoreHandle spiTxDMAComplete;
static xSemaphoreHandle spiRxDMAComplete;

static bool isInit;

bool icm20789EvaluateSelfTest(float low, float high, float value, char* string);

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RZR_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(RZR_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RZR_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(RZR_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

static void writeReg(uint8_t reg, uint8_t val)
{
  RZR_EN_CS();
  spiSendByte(reg);
  spiSendByte(val);
  RZR_DIS_CS();
}

static uint8_t readReg(uint8_t reg)
{
  uint8_t regval;
  RZR_EN_CS();
  spiSendByte(reg | 0x80);
  regval = spiReceiveByte();
  RZR_DIS_CS();

  return regval;
}

static void readRegPar(uint8_t reg, uint8_t *val)
{
  RZR_EN_CS();
  spiSendByte(reg | 0x80);
  *val = spiReceiveByte();
  RZR_DIS_CS();
}

//static void readSensorRegs(void)
//{
//  static uint8_t regs[6];
//
//  RZR_EN_CS();
//  spiSendByte(MPU6500_RA_GYRO_XOUT_H | 0x80);
//
//  for (int i=0; i<6; i++)
//  {
//    regs[i] = spiReceiveByte();
//    consolePrintf("%X ", regs[i]);
//  }
//  consolePutchar('\n');
//  RZR_DIS_CS();
//}


static void spiRzrConfigureSlow(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(RZR_SPI, DISABLE);
  SPI_I2S_DeInit(RZR_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //~0.66 MHz
  SPI_Init(RZR_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(RZR_SPI, ENABLE);
}

static void spiRzrConfigureFast(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(RZR_SPI, DISABLE);
  SPI_I2S_DeInit(RZR_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //~10.5 MHz
  SPI_Init(RZR_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(RZR_SPI, ENABLE);
}

/* Initialisation */
static void spiInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(RZR_GPIO_SPI_CLK | RZR_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(RZR_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_SPI_SCK |  RZR_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RZR_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(RZR_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RZR_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(RZR_GPIO_CS_PORT, &GPIO_InitStructure);

  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_SCK_SRC, RZR_SPI_AF);
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_MISO_SRC, RZR_SPI_AF);
  GPIO_PinAFConfig(RZR_GPIO_SPI_PORT, RZR_GPIO_SPI_MOSI_SRC, RZR_SPI_AF);

  /* disable the chip select */
  RZR_DIS_CS();

  spiRzrConfigureSlow();
}

static void spiDMAInit(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  RZR_SPI_DMA_CLK_INIT(RZR_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(RZR_SPI->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = RZR_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(RZR_SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(RZR_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = RZR_SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(RZR_SPI_RX_DMA_STREAM, DISABLE);
  DMA_Init(RZR_SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = RZR_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = RZR_SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}

bool icm20789Init(void)
{
  if (isInit)
    return false;

  spiInit();
  spiDMAInit();

  spiTxDMAComplete = xSemaphoreCreateBinary();
  spiRxDMAComplete = xSemaphoreCreateBinary();

  if (readReg(MPU6500_RA_WHO_AM_I) == 0x03)
  {
    writeReg(MPU6500_RA_PWR_MGMT_1, 0x80);           // Reset device
    vTaskDelay(M2T(100));
    writeReg(MPU6500_RA_PWR_MGMT_1, 0x03);           // ZGYRO as clock source
    vTaskDelay(M2T(10));
    writeReg(MPU6500_RA_USER_CTRL, 0b00011101);      // Reset signal path and i2c disable
    vTaskDelay(M2T(100));
    writeReg(MPU6500_RA_GYRO_CONFIG, (0x03 << 3));   // 2000 deg/s & 250Hz DLPF
    writeReg(MPU6500_RA_ACCEL_CONFIG, (0x03 << 3));  // 16G

    writeReg(MPU6500_RA_INT_PIN_CFG, 0b00010000);    // Clear on any read.
    writeReg(MPU6500_RA_INT_ENABLE,  0b00000001);    // Enable raw sensor data interrupt

    spiRzrConfigureFast(); // Switch to 10.5Mhz
    isInit = true;
  }

  return isInit;
}

bool icm20789Test(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus = icm20789TestConnection();

  return testStatus;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool icm20789TestConnection(void)
{
  return (readReg(MPU6500_RA_WHO_AM_I) == 0x03);
}

/** Do a MPU6500 self test.
 * @return True if self test passed, false otherwise
 */
bool icm20789SelfTest()
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t saveReg[5];
  uint8_t selfTest[6];
  int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
  int32_t factoryTrim[6];
  float aDiff[3], gDiff[3];
  uint8_t FS = 0;
  int i;

  // Save old configuration
  readRegPar(MPU6500_RA_SMPLRT_DIV, &saveReg[0]);
  readRegPar(MPU6500_RA_CONFIG, &saveReg[1]);
  readRegPar(MPU6500_RA_GYRO_CONFIG, &saveReg[2]);
  readRegPar(MPU6500_RA_ACCEL_CONFIG_2, &saveReg[3]);
  readRegPar(MPU6500_RA_ACCEL_CONFIG, &saveReg[4]);
  // Write test configuration
  writeReg(MPU6500_RA_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
  writeReg(MPU6500_RA_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeReg(MPU6500_RA_GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
  writeReg(MPU6500_RA_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeReg(MPU6500_RA_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for(i = 0; i < 200; i++)
  {
    // get average current values of gyro and acclerometer
//FIXME    i2cdevRead(I2Cx, devAddr, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    //FIXME    i2cdevRead(I2Cx, devAddr, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
    gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
  }

  for (i = 0; i < 3; i++)
  { // Get average of 200 values and store as average current readings
    aAvg[i] /= 200;
    gAvg[i] /= 200;
  }

  // Configure the accelerometer for self-test
  writeReg(MPU6500_RA_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeReg(MPU6500_RA_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  vTaskDelay(M2T(25)); // Delay a while to let the device stabilize

  for(i = 0; i < 200; i++)
  {
    // get average self-test values of gyro and acclerometer
    //FIXME     i2cdevRead(I2Cx, devAddr, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    //FIXME i2cdevRead(I2Cx, devAddr, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (i =0; i < 3; i++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[i] /= 200;
    gSTAvg[i] /= 200;
  }

   // Configure the gyro and accelerometer for normal operation
   writeReg(MPU6500_RA_ACCEL_CONFIG, 0x00);
   writeReg(MPU6500_RA_GYRO_CONFIG, 0x00);
   vTaskDelay(M2T(25)); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   readRegPar(MPU6500_RA_ST_X_ACCEL, &selfTest[0]); // X-axis accel self-test results
   readRegPar(MPU6500_RA_ST_Y_ACCEL, &selfTest[1]); // Y-axis accel self-test results
   readRegPar(MPU6500_RA_ST_Z_ACCEL, &selfTest[2]); // Z-axis accel self-test results
   readRegPar(MPU6500_RA_ST_X_GYRO, &selfTest[3]); // X-axis gyro self-test results
   readRegPar(MPU6500_RA_ST_Y_GYRO, &selfTest[4]); // Y-axis gyro self-test results
   readRegPar(MPU6500_RA_ST_Z_GYRO, &selfTest[5]); // Z-axis gyro self-test results

   for (i = 0; i < 6; i++)
   {
      if (selfTest[i] != 0)
      {
//        factoryTrim[i] = icm20789StTb[selfTest[i] - 1];
      }
      else
      {
        factoryTrim[i] = 0;
      }
    }

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (i = 0; i < 3; i++)
  {
   aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i]) - factoryTrim[i]))/factoryTrim[i]; // Report percent differences
   gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i]) - factoryTrim[i+3]))/factoryTrim[i+3]; // Report percent differences
//   DEBUG_PRINT("a[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, aAvg[i], aSTAvg[i], aSTAvg[i] - aAvg[i], factoryTrim[i], aDiff[i]);
//   DEBUG_PRINT("g[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, gAvg[i], gSTAvg[i], gSTAvg[i] - gAvg[i], factoryTrim[i+3], gDiff[i]);
  }

  // Restore old configuration
  writeReg(MPU6500_RA_SMPLRT_DIV, saveReg[0]);
  writeReg(MPU6500_RA_CONFIG, saveReg[1]);
  writeReg(MPU6500_RA_GYRO_CONFIG, saveReg[2]);
  writeReg(MPU6500_RA_ACCEL_CONFIG_2, saveReg[3]);
  writeReg(MPU6500_RA_ACCEL_CONFIG, saveReg[4]);

   // Check result
  if (icm20789EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[0], "gyro X") &&
      icm20789EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[1], "gyro Y") &&
      icm20789EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[2], "gyro Z") &&
      icm20789EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[0], "acc X") &&
      icm20789EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[1], "acc Y") &&
      icm20789EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[2], "acc Z"))
  {
    return true;
  }
  else
  {
    return false;
  }
}

/** Evaluate the values from a MPU6500 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
bool icm20789EvaluateSelfTest(float low, float high, float value, char* string)
{
  if (value < low || value > high)
  {
    DEBUG_PRINT("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)low, (double)high, (double)value);
    return false;
  }
  return true;
}

void icm20789ReadAllSensors(uint8_t *buffer)
{
  // Disable peripheral before setting up for duplex DMA
  SPI_Cmd(RZR_SPI, DISABLE);

  RZR_EN_CS();

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = SENSORS_READ_START_CMD;
  RZR_SPI_TX_DMA_STREAM->M0AR = (uint32_t)&spiTxBuffer[0];
  RZR_SPI_TX_DMA_STREAM->NDTR = SENSORS_READ_TRANSACTION_SIZE;

  RZR_SPI_RX_DMA_STREAM->M0AR = (uint32_t)&spiRxBuffer[0];
  RZR_SPI_RX_DMA_STREAM->NDTR = SENSORS_READ_TRANSACTION_SIZE;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(RZR_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(RZR_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(RZR_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(RZR_SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  // Enable DMA Streams
  DMA_Cmd(RZR_SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(RZR_SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(RZR_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(RZR_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  SPI_Cmd(RZR_SPI, ENABLE);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it in sensors_icm20789.c
  memcpy(buffer, &spiRxBuffer[1], SENSORS_READ_TRANSACTION_SIZE - 1);

  RZR_DIS_CS();
}

void __attribute__((used)) RZR_SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(RZR_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(RZR_SPI_TX_DMA_STREAM, RZR_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(RZR_SPI_TX_DMA_STREAM,RZR_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(RZR_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(RZR_SPI_TX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) RZR_SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(RZR_SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(RZR_SPI_RX_DMA_STREAM, RZR_SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(RZR_SPI_RX_DMA_STREAM, RZR_SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(RZR_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(RZR_SPI_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
