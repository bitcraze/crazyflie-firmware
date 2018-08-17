/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 * sensors_bmi088_bmp388.c: IMU sensor accusation for the *88 bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include "sensors_bosch.h"
#include "i2cdev.h"

#define SENSORS_TAKE_ACCEL_BIAS

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define BMI088_ACC_GPIO_CS             GPIO_Pin_1
#define BMI088_ACC_GPIO_CS_PORT        GPIOB
#define BMI088_ACC_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOB
#define BMI088_GYR_GPIO_CS             GPIO_Pin_0
#define BMI088_GYR_GPIO_CS_PORT        GPIOB
#define BMI088_GYR_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOB

#define BMI088_SPI                     SPI2
#define BMI088_SPI_AF                  GPIO_AF_SPI2
#define BMI088_SPI_CLK                 RCC_APB1Periph_SPI2
#define BMI088_GPIO_SPI_PORT           GPIOB
#define BMI088_GPIO_SPI_CLK            RCC_AHB1Periph_GPIOB
#define BMI088_GPIO_SPI_SCK            GPIO_Pin_13
#define BMI088_GPIO_SPI_SCK_SRC        GPIO_PinSource13
#define BMI088_GPIO_SPI_MISO           GPIO_Pin_14
#define BMI088_GPIO_SPI_MISO_SRC       GPIO_PinSource14
#define BMI088_GPIO_SPI_MOSI           GPIO_Pin_15
#define BMI088_GPIO_SPI_MOSI_SRC       GPIO_PinSource15

#define BMI088_SPI_DMA_IRQ_PRIO        NVIC_HIGH_PRI
#define BMI088_SPI_DMA                 DMA1
#define BMI088_SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define BMI088_SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define BMI088_SPI_RX_DMA_STREAM       DMA1_Stream3
#define BMI088_SPI_RX_DMA_IRQ          DMA1_Stream3_IRQn
#define BMI088_SPI_RX_DMA_IRQHandler   DMA1_Stream3_IRQHandler
#define BMI088_SPI_RX_DMA_CHANNEL      DMA_Channel_0
#define BMI088_SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define BMI088_SPI_TX_DMA_STREAM       DMA1_Stream4
#define BMI088_SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define BMI088_SPI_TX_DMA_IRQHandler   DMA1_Stream4_IRQHandler
#define BMI088_SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define BMI088_SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define DUMMY_BYTE    0x00

/* Usefull macro */
#define ACC_EN_CS() GPIO_ResetBits(BMI088_ACC_GPIO_CS_PORT, BMI088_ACC_GPIO_CS)
#define ACC_DIS_CS() GPIO_SetBits(BMI088_ACC_GPIO_CS_PORT, BMI088_ACC_GPIO_CS)
#define GYR_EN_CS() GPIO_ResetBits(BMI088_GYR_GPIO_CS_PORT, BMI088_GYR_GPIO_CS)
#define GYR_DIS_CS() GPIO_SetBits(BMI088_GYR_GPIO_CS_PORT, BMI088_GYR_GPIO_CS)

/* Defines and buffers for full duplex SPI DMA transactions */
#define SPI_MAX_DMA_TRANSACTION_SIZE    15 // 1 byte command followed by 14 bytes data
static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static xSemaphoreHandle spiTxDMAComplete;
static xSemaphoreHandle spiRxDMAComplete;


typedef struct {
  Axis3i16      value;
  Axis3i16*     bufStart;
  Axis3i16*     bufPtr;
  uint8_t       found : 1;
  uint8_t       ongoing : 1;
  uint8_t       bufIsFull : 1;
} BiasObj;

/* initialize necessary variables */
static struct bmi088_dev bmi088Dev;
static struct bmp3_dev   bmp388Dev;

static xQueueHandle accelDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magDataQueue;
static xQueueHandle baroDataQueue;
static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit = false;
static bool allSensorsAreCalibrated = false;
static sensorData_t sensorData;
static uint64_t imuIntTimestamp;

static int32_t varianceSampleTime;
static uint8_t sensorsAccLpfAttFactor;

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

static void sensorsDeviceInit(void);
static void sensorsTaskInit(void);
static void sensorsTask(void *param);
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale);
static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature);
static bool processGyroBias(BiasObj* bias);
static void processAccelBias(BiasObj* bias);

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);
static void sensorsBiasReset(BiasObj* bias);
static void sensorsBiasMalloc(BiasObj* bias);
static void sensorsBiasFree(BiasObj* bias);
static void sensorsBiasBufPtrIncrement(BiasObj* bias);

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI peripheral */
  SPI_I2S_SendData(BMI088_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(BMI088_SPI);
}

static char spiReceiveByte()
{
  return spiSendByte(DUMMY_BYTE);
}

static void spiDMATransaction(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  ASSERT(len < SPI_MAX_DMA_TRANSACTION_SIZE);

  // Disable peripheral before setting up for duplex DMA
  SPI_Cmd(BMI088_SPI, DISABLE);

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = reg_addr;
  BMI088_SPI_TX_DMA_STREAM->M0AR = (uint32_t)&spiTxBuffer[0];
  BMI088_SPI_TX_DMA_STREAM->NDTR = len + 1;

  BMI088_SPI_RX_DMA_STREAM->M0AR = (uint32_t)&spiRxBuffer[0];
  BMI088_SPI_RX_DMA_STREAM->NDTR = len + 1;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(BMI088_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(BMI088_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(BMI088_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|
                DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(BMI088_SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|
                DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  // Enable DMA Streams
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  SPI_Cmd(BMI088_SPI, ENABLE);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}

// Communication routines

static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  /**< Burst read code comes here */
  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_EN_CS();
  }
  else
  {
    GYR_EN_CS();
  }

  if (len <= 1 || len > SPI_MAX_DMA_TRANSACTION_SIZE)
  {
    spiSendByte(reg_addr);
    for (int i = 0; i < len; i++)
    {
      reg_data[i] = spiReceiveByte();
    }
  }
  else
  {
    spiDMATransaction(reg_addr, reg_data, len);
  }

  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_DIS_CS();
  }
  else
  {
    GYR_DIS_CS();
  }

  return BSTDR_OK;
}

static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_EN_CS();
  }
  else
  {
    GYR_EN_CS();
  }
  spiSendByte(reg_addr);
  for (int i = 0; i < len; i++)
  {
    spiSendByte(reg_data[i]);
  }

  if (dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY)
  {
    ACC_DIS_CS();
  }
  else
  {
    GYR_DIS_CS();
  }

  return BSTDR_OK;
}

/***********************
 * I2C private methods *
 ***********************/
static bstdr_ret_t i2c_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  /**< Burst read code comes here */
  if (i2cdevRead(I2C3_DEV, dev_id, reg_addr, (uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}

static bstdr_ret_t i2c_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  /**< Burst write code comes here */
  if (i2cdevWrite(I2C3_DEV, dev_id,reg_addr,(uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}


static void bmi088_ms_delay(uint32_t period)
{
  /**< Delay code comes */
  vTaskDelay(M2T(period));
}

static void spiConfigure(void)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(BMI088_SPI, DISABLE);
  SPI_I2S_DeInit(BMI088_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //~10.5 MHz
  SPI_Init(BMI088_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(BMI088_SPI, ENABLE);
}

/* Initialisation */
static void spiInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(BMI088_GPIO_SPI_CLK | BMI088_ACC_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(BMI088_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = BMI088_GPIO_SPI_SCK |  BMI088_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BMI088_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = BMI088_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(BMI088_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = BMI088_ACC_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BMI088_ACC_GPIO_CS_PORT, &GPIO_InitStructure);
  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = BMI088_GYR_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BMI088_GYR_GPIO_CS_PORT, &GPIO_InitStructure);


  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_SCK_SRC, BMI088_SPI_AF);
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_MISO_SRC, BMI088_SPI_AF);
  GPIO_PinAFConfig(BMI088_GPIO_SPI_PORT, BMI088_GPIO_SPI_MOSI_SRC, BMI088_SPI_AF);

  /* disable the chip select */
  ACC_DIS_CS();

  spiConfigure();
}

static void spiDMAInit(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  BMI088_SPI_DMA_CLK_INIT(BMI088_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(BMI088_SPI->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = BMI088_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(BMI088_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = BMI088_SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM, DISABLE);
  DMA_Init(BMI088_SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = BMI088_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = BMI088_SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  spiTxDMAComplete = xSemaphoreCreateBinary();
  spiRxDMAComplete = xSemaphoreCreateBinary();
}

static void sensorsInterruptInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  sensorsDataReady = xSemaphoreCreateBinary();
  dataReady = xSemaphoreCreateBinary();

  // Enable the interrupt on PC14
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line14);
  portENABLE_INTERRUPTS();
}

void sensorsInit(void)
{
  if (isInit)
    {
      return;
    }

  i2cdevInit(I2C3_DEV);
  spiInit();
  spiDMAInit();

  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();
}

static void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  /* BMI088 */
  bmi088Dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
  bmi088Dev.gyro_id = BMI088_GYRO_I2C_ADDR_PRIMARY;
  bmi088Dev.interface = BMI088_SPI_INTF;
  bmi088Dev.read = spi_burst_read;
  bmi088Dev.write = spi_burst_write;
  bmi088Dev.delay_ms = bmi088_ms_delay;

  /* BMI088 GYRO */
  rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    struct bmi088_int_cfg intConfig;

    DEBUG_PRINT("BMI088 Gyro SPI connection [OK].\n");
    /* set power mode of gyro */
    bmi088Dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
    rslt |= bmi088_set_gyro_power_mode(&bmi088Dev);
    /* set bandwidth and range of gyro */
    bmi088Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    bmi088Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
    bmi088Dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
    rslt |= bmi088_set_gyro_meas_conf(&bmi088Dev);

    intConfig.gyro_int_channel = BMI088_INT_CHANNEL_3;
    intConfig.gyro_int_type = BMI088_GYRO_DATA_RDY_INT;
    intConfig.gyro_int_pin_3_cfg.enable_int_pin = 1;
    intConfig.gyro_int_pin_3_cfg.lvl = 1;
    intConfig.gyro_int_pin_3_cfg.output_mode = 0;
    /* Setting the interrupt configuration */
    rslt = bmi088_set_gyro_int_config(&intConfig, &bmi088Dev);

    bmi088Dev.delay_ms(50);
    struct bmi088_sensor_data gyr;
    rslt |= bmi088_get_gyro_data(&gyr, &bmi088Dev);

  }
  else
  {
#ifndef SENSORS_IGNORE_IMU_FAIL
    DEBUG_PRINT("BMI088 Gyro SPI connection [FAIL]\n");
    isInit = false;
#endif
  }

  /* BMI088 ACCEL */
  rslt |= bmi088_accel_switch_control(&bmi088Dev, BMI088_ACCEL_POWER_ENABLE);
  bmi088Dev.delay_ms(5);

  rslt = bmi088_accel_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    DEBUG_PRINT("BMI088 Accel SPI connection [OK]\n");
    /* set power mode of accel */
    bmi088Dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
    rslt |= bmi088_set_accel_power_mode(&bmi088Dev);
    bmi088Dev.delay_ms(10);

    /* set bandwidth and range of accel */
    bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
    bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
    bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
    rslt |= bmi088_set_accel_meas_conf(&bmi088Dev);

    struct bmi088_sensor_data acc;
    rslt |= bmi088_get_accel_data(&acc, &bmi088Dev);
  }
  else
  {
#ifndef SENSORS_IGNORE_IMU_FAIL
    DEBUG_PRINT("BMI088 Accel SPI connection [FAIL]\n");
    isInit = false;
#endif
  }

  /* BMP388 */
  bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = i2c_burst_read;
  bmp388Dev.write = i2c_burst_write;
  bmp388Dev.delay_ms = bmi088_ms_delay;

  int i = 3;
  do {
    bmp388Dev.delay_ms(1);
    // For some reason it often doesn't work first time
    rslt = bmp3_init(&bmp388Dev);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK)
  {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* Select the pressure and temperature sensor to be enabled */
    bmp388Dev.settings.press_en = BMP3_ENABLE;
    bmp388Dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

    /* Set the power mode to normal mode */
    bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp388Dev);


    bmp388Dev.delay_ms(20); // wait before first read out
    // read out data
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    /* Print the temperature and pressure data */
//    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
  else
  {
#ifndef SENSORS_IGNORE_BAROMETER_FAIL
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    isInit = false;
    return;
#endif
  }

  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  sensorsAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void)
{
  accelDataQueue = xQueueCreate(1, sizeof(Axis3f));
  gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
  magDataQueue = xQueueCreate(1, sizeof(Axis3f));
  baroDataQueue = xQueueCreate(1, sizeof(baro_t));

  xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE,
              NULL, SENSORS_TASK_PRI, NULL);
}

static void sensorsGyroGet(Axis3i16* dataOut) {
  bmi088_get_gyro_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsAccelGet(Axis3i16* dataOut) {
  bmi088_get_accel_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsGyroCalibrate(BiasObj* gyro) {
  if (gyro->found == 0)
    {
      if (gyro->ongoing == 0)
        {
          sensorsBiasMalloc(gyro);
        }
      /* write directly into buffer */
      sensorsGyroGet(gyro->bufPtr);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(gyro);

      if (gyro->bufIsFull == 1)
        {
          if (processGyroBias(gyro))
            sensorsBiasFree(gyro);
        }
    }
}

static void __attribute__((used))
sensorsAccelCalibrate(BiasObj* accel, BiasObj* gyro) {
  if (accel->found == 0)
    {
      if (accel->ongoing == 0)
        {
          sensorsBiasMalloc(accel);
        }
      /* write directly into buffer */
      sensorsAccelGet(accel->bufPtr);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(accel);
      if ( (accel->bufIsFull == 1) && (gyro->found == 1) )
      {
        processAccelBias(accel);
        accel->value.z -= SENSORS_BMI088_1G_IN_LSB;
        sensorsBiasFree(accel);
      }
    }
}

static void sensorsTask(void *param)
{
  systemWaitStart();

//  uint32_t lastWakeTime = xTaskGetTickCount();
  static BiasObj bmi088GyroBias;
#ifdef SENSORS_TAKE_ACCEL_BIAS
  static BiasObj bmi088AccelBias;
#endif
  Axis3i16 gyro;
  Axis3i16 accel;
  Axis3f accelScaled;
  Axis3i16 accelLPF;
  Axis3i32 accelStoredFilterValues;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1)
  {
//      vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      sensorData.interruptTimestamp = imuIntTimestamp;
      /* calibrate if necessary */
      if (!allSensorsAreCalibrated)
      {
        if (!bmi088GyroBias.found)
        {
          sensorsGyroCalibrate(&bmi088GyroBias);
  #ifdef SENSORS_TAKE_ACCEL_BIAS
          sensorsAccelCalibrate(&bmi088AccelBias,
                                &bmi088GyroBias);
  #endif
        }

        if (bmi088GyroBias.found
  #ifdef SENSORS_TAKE_ACCEL_BIAS
            && bmi088AccelBias.found
  #endif
        )
        {
          // soundSetEffect(SND_CALIB);
          DEBUG_PRINT("Sensor calibration [OK].\n");
          ledseqRun(SYS_LED, seq_calibrated);
          allSensorsAreCalibrated = true;
        }
      }
    else
    {
      /* get data from chosen sensors */
      sensorsGyroGet(&gyro);
      sensorsAccelGet(&accel);
      sensorsApplyBiasAndScale(&sensorData.gyro, &gyro,
                               &bmi088GyroBias.value,
                               SENSORS_BMI088_DEG_PER_LSB_CFG);

      sensorsAccIIRLPFilter(&accel, &accelLPF,
                            &accelStoredFilterValues,
                            (int32_t)sensorsAccLpfAttFactor);
      sensorsApplyBiasAndScale(&accelScaled, &accelLPF,
                               &bmi088AccelBias.value,
                               SENSORS_BMI088_G_PER_LSB_CFG);

      sensorsAccAlignToGravity(&accelScaled, &sensorData.acc);

      }
      // TODO: Move barometer reading to separate task to minimize gyro to output latency
      if (isBarometerPresent)
      {
        static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
        if (--baroMeasDelay == 0)
        {
          uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
          struct bmp3_data data;
          baro_t* baro388 = &sensorData.baro;
          /* Temperature and Pressure data are read and stored in the bmp3_data instance */
          bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
          sensorsScaleBaro(baro388, data.pressure, data.temperature);
          baroMeasDelay = baroMeasDelayMin;
        }
      }

      xQueueOverwrite(accelDataQueue, &sensorData.acc);
      xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
      if (isBarometerPresent)
      {
        xQueueOverwrite(baroDataQueue, &sensorData.baro);
      }

      xSemaphoreGive(dataReady);
    }
  }
}

void sensorsWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsBiasMalloc(BiasObj* bias)
{
  /* allocate memory for buffer */
  bias->bufStart =
      pvPortMalloc(SENSORS_NBR_OF_BIAS_SAMPLES * sizeof(Axis3i16));
  bias->bufPtr = bias->bufStart;
  /* set ongoing bit */
  bias->ongoing = 1;
}

static void __attribute__((used)) sensorsBiasReset(BiasObj* bias)
{
  /* unset bias found and buffer full status bits */
  bias->found = 0;
  bias->bufIsFull = 0;
  /* set bufPtr to start to ensure the buffer has to be refilled
   * completly before bufferIsFull Status bit will be set */
  bias->bufPtr = bias->bufStart;
  /* clear any exisiting bias value */
  bias->value.x = 0;
  bias->value.y = 0;
  bias->value.z = 0;
  allSensorsAreCalibrated = false;
}

static void sensorsBiasFree(BiasObj* bias)
{
  /* unset buffer is full */
  bias->bufIsFull = 0;
  /* free buffer memory */
  vPortFree(bias->bufStart);
  bias->bufStart = NULL;
  bias->bufPtr = NULL;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsBiasBufPtrIncrement(BiasObj* bias)
{
  bias->bufPtr++;
  if (bias->bufPtr >= bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES)
    {
      bias->bufPtr = bias->bufStart;
      bias->bufIsFull = 1;
    }
}

/**
 * Calculates the mean for the bias buffer.
 */
static void calcMean(BiasObj* bias, Axis3f* mean) {
  Axis3i16* elem;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
       elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sum[0] += elem->x;
      sum[1] += elem->y;
      sum[2] += elem->z;
    }

  mean->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void calcVarianceAndMean(BiasObj* bias, Axis3f* variance, Axis3f* mean)
{
  Axis3i16* elem;
  int64_t sumSquared[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
      elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sumSquared[0] += elem->x * elem->x;
      sumSquared[1] += elem->y * elem->y;
      sumSquared[2] += elem->z * elem->z;
    }
  calcMean(bias, mean);

  variance->x = fabs(sumSquared[0] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->x * mean->x);
  variance->y = fabs(sumSquared[1] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->y * mean->y);
  variance->z = fabs(sumSquared[2] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->z * mean->z);
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool processGyroBias(BiasObj* bias)
{
  Axis3f mean, variance;
  calcVarianceAndMean(bias, &variance, &mean);

  if (variance.x < GYRO_VARIANCE_THRESHOLD_X
      && variance.y < GYRO_VARIANCE_THRESHOLD_Y
      && variance.z < GYRO_VARIANCE_THRESHOLD_Z
      && (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->value.x = (int16_t)(mean.x + 0.5f);
      bias->value.y = (int16_t)(mean.y + 0.5f);
      bias->value.z = (int16_t)(mean.z + 0.5f);
      bias->found = 1;
      return true;
    }
  return false;
}

static void processAccelBias(BiasObj* bias)
{
  Axis3f mean;
  calcMean(bias, &mean);

  varianceSampleTime = xTaskGetTickCount();
  bias->value.x = (int16_t)(mean.x + 0.5f);
  bias->value.y = (int16_t)(mean.y + 0.5f);
  bias->value.z = (int16_t)(mean.z + 0.5f);
  bias->found = 1;
}
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale) {
  scaled->x = ((float)aligned->x - (float)bias->x) * scale;
  scaled->y = ((float)aligned->y - (float)bias->y) * scale;
  scaled->z = ((float)aligned->z - (float)bias->z) * scale;
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature) {
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelDataQueue, acc, 0));
}

bool sensorsReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magDataQueue, mag, 0));
}

bool sensorsReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(baroDataQueue, baro, 0));
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  zRangerReadRange(&sensors->zrange, tick);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsAreCalibrated()
{
  return allSensorsAreCalibrated;
}

bool sensorsTest(void)
{
  bool testStatus = true;

  if (!isInit)
    {
      DEBUG_PRINT("Uninitialized\n");
      testStatus = false;
    }

  return testStatus;
}

bool sensorsManufacturingTest(void)
{
  return true;
}

bool sensorsHasBarometer(void)
{
  return isBarometerPresent;
}

bool sensorsHasMangnetometer(void)
{
  return isMagnetometerPresent;
}

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void __attribute__((used)) EXTI14_Callback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) BMI088_SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI088_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI088_SPI_TX_DMA_STREAM, BMI088_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI088_SPI_TX_DMA_STREAM,BMI088_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI088_SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) BMI088_SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI088_SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI088_SPI_RX_DMA_STREAM, BMI088_SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI088_SPI_RX_DMA_STREAM, BMI088_SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI088_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI088_SPI_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}


PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP388, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
