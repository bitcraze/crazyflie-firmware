
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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 * 
 * Bidirectional DSHOT telemetry driver based on code from:
 *     R. Busetto, E. Cereda, M. Forgione, G. Maroni, D. Piga, D. Palossi,
 *     "Nonlinear System Identification Nano-drone Benchmark",
 *     arXiv preprint arXiv:2512.14450, 2025. https://arxiv.org/abs/2512.14450
 * If you find it useful for your research, academic citations are appreciated.
 * 
 */
#define DEBUG_MODULE "MTR-DRV"

#include <stdbool.h>
#include <math.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"
#include "debug.h"
#include "power_distribution.h"
#include "nvicconf.h"
#include "usec_time.h"
#include "platform_defaults.h"
//FreeRTOS includes
#include "task.h"

//Logging includes
#include "log.h"
#include "param.h"

static uint8_t motorSetEnable = 0;
static uint16_t motorPowerSet[] = {0, 0, 0, 0}; // user-requested PWM signals (overrides)
static uint16_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static DMA_InitTypeDef DMA_InitStructureShare;
static bool doResetESCs = true;
// Memory buffer for DSHOT bits
static uint32_t dshotDmaBuffer[NBR_OF_MOTORS][DSHOT_DMA_BUFFER_SIZE];
static void motorsDshotSetup();
static void motorsDshotOutputSetup(int id);
static volatile uint32_t dmaWait;

#define DSHOT_TELEMETRY_INVALID         (UINT16_MAX)
#define DSHOT_ERPM_INVALID              (UINT16_MAX)

typedef enum {
  DSHOT_STATE_IDLE = 0,
  DSHOT_STATE_OUTPUT,
  DSHOT_STATE_INPUT,
} dshotState_t;

static dshotState_t dshotState[NBR_OF_MOTORS] = {
  DSHOT_STATE_IDLE, DSHOT_STATE_IDLE, DSHOT_STATE_IDLE, DSHOT_STATE_IDLE
};

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
static uint8_t motorNbrOfPoles = CONFIG_MOTORS_NBR_OF_POLES;

static uint32_t dshotDmaInputBuffer[NBR_OF_MOTORS][DSHOT_TELEMETRY_MAX_GCR_EDGES];
static uint16_t dshotTelemetryPackets[NBR_OF_MOTORS] = {
  DSHOT_TELEMETRY_INVALID, DSHOT_TELEMETRY_INVALID, DSHOT_TELEMETRY_INVALID, DSHOT_TELEMETRY_INVALID
};
static uint16_t motorRPMs[NBR_OF_MOTORS] = {
  DSHOT_ERPM_INVALID, DSHOT_ERPM_INVALID, DSHOT_ERPM_INVALID, DSHOT_ERPM_INVALID
};
#endif
#endif

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#include "motors_def.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

const MotorHealthTestDef brushedMotorHealthTestSettings = {
   .onPeriodMsec = HEALTH_BRUSHED_ON_PERIOD_MSEC,
   .offPeriodMsec = HEALTH_BRUSHED_OFF_PERIOD_MSEC,
   .varianceMeasurementStartMsec = HEALTH_BRUSHED_VARIANCE_START_MSEC,
   .onPeriodPWMRatioProp = HEALTH_BRUSHED_PROP_ON_PERIOD_PWM_RATIO,
   .onPeriodPWMRatioBat = HEALTH_BRUSHED_BAT_ON_PERIOD_PWM_RATIO,
};

const MotorHealthTestDef brushlessMotorHealthTestSettings = {
    .onPeriodMsec = HEALTH_BRUSHLESS_ON_PERIOD_MSEC,
    .offPeriodMsec = HEALTH_BRUSHLESS_OFF_PERIOD_MSEC,
    .varianceMeasurementStartMsec = HEALTH_BRUSHLESS_VARIANCE_START_MSEC,
    .onPeriodPWMRatioProp = 0, /* user must set health.propTestPWMRatio explicitly */
    .onPeriodPWMRatioBat = 0, /* user must set health.batTestPWMRatio explicitly */
};

const MotorHealthTestDef unknownMotorHealthTestSettings = {
    .onPeriodMsec = 0,
    .offPeriodMsec = 0,
    .varianceMeasurementStartMsec = 0,
    .onPeriodPWMRatioProp = 0,
    .onPeriodPWMRatioBat = 0,
};

static bool isInit = false;
static uint64_t lastCycleTime;
static uint32_t cycleTime;


/* Private functions */

#ifndef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}
#endif

static uint16_t motorsConv16ToBits(uint16_t bits)
{
  return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

GPIO_InitTypeDef GPIO_PassthroughInput =
{
    .GPIO_Mode = GPIO_Mode_IN,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_OD,
    .GPIO_PuPd = GPIO_PuPd_UP
};

GPIO_InitTypeDef GPIO_PassthroughOutput =
{
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP
};

// What decides the amount of thrust is the voltage of the motor, which is calculated by
// duty cycle * supply voltage.
// At a certain thrust command, there needs to be a corresponding motor voltage to 
// achieve that thrust. That mapping is here fit with a third order polynomial, because
// the thrust is correlated quadratic with the rpm, but there also is some saturation
// taking place in a nonideal brushed DC motor with a quadratic load.
//
// In this case however, we have the thrust given and need to calculate the necessary 
// motor voltage. For that reason, an inversion of the mapping is done below.
// To achieve a motor voltage, the duty cycle needs to be adapted based on 
// the given supply voltage, see the above eq. 
//
// For more information see the corresponding blog entry on "PWM to Thrust"
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage)
{
  #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
  ASSERT(id < NBR_OF_MOTORS);

  /*
  * A LiPo battery is supposed to be 4.2V charged, 3.7V mid-charge and 3V
  * discharged.
  *
  * A suitable sanity check for disabling the voltage compensation would be
  * under 2V. That would suggest a damaged battery. This protects against
  * rushing the motors on bugs and invalid voltage levels.
  */
  if (supplyVoltage < 2.0f)
  {
    return 0.0f;
  }

  float thrust = (iThrust / 65535.0f) * THRUST_MAX; // rescaling integer thrust to N
  if (thrust < THRUST_MIN)                          // Make sure inversion is unique
  {
    return 0.0f;
  }
  else 
  {
    // Motor voltage to thrust is a cubic fit
    // q, r, p to calculate the inverse of the third order polynomial
    // For more info see https://math.vanderbilt.edu/schectex/courses/cubic/
    // q and thus qrp need to be calculated each time while p and r are constant
    static const float p = -VMOTOR2THRUST2 / (3 * VMOTOR2THRUST3);
    float q = p * p * p + (VMOTOR2THRUST2 * VMOTOR2THRUST1 - 3 * VMOTOR2THRUST3 * (VMOTOR2THRUST0 - thrust)) / (6 * VMOTOR2THRUST3 * VMOTOR2THRUST3);
    static const float r = VMOTOR2THRUST1 / (3 * VMOTOR2THRUST3);
    float qrp = sqrtf(q * q + (r - p * p) * (r - p * p) * (r - p * p));

    float motorVoltage = cbrtf(q + qrp) + cbrtf(q - qrp) + p;
    float ratio = motorVoltage / supplyVoltage;
    return UINT16_MAX * ratio;
  }

  #endif

  return iThrust;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%.
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    // First to init will configure it
    return;
  }

  motorMap = motorMapSelect;

  DEBUG_PRINT("Using %s motor driver\n", motorMap[0]->drvType == BRUSHED ? "brushed" : "brushless");

  if (motorMap[MOTOR_M1]->hasPC15ESCReset)
  {
    MOTORS_RCC_GPIO_CMD(RCC_AHB1Periph_GPIOC, ENABLE);
    // Don't reset just yet but when we start sending DSHOT signals
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
    // Configure the GPIO for CF-BL ESC RST
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
  }

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPowerswitchPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // If there is a power switch, as on Bolt, enable power to ESC by
    // switching on mosfet.
    if (motorMap[i]->gpioPowerswitchPin != 0)
    {
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPowerswitchPin;
      GPIO_Init(motorMap[i]->gpioPowerswitchPort, &GPIO_InitStructure);
      GPIO_WriteBit(motorMap[i]->gpioPowerswitchPort, motorMap[i]->gpioPowerswitchPin, 1);
    }

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    // Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(motorMap[i]->tim, DISABLE); //TODO: Why?

  uint16_t timPolarity = motorMap[i]->timPolarity;
  #ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
      // For bidirectional DSHOT we need active high PWM
      timPolarity = (timPolarity == TIM_OCPolarity_High) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
  #endif
    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    // Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);
  }

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
  motorsDshotSetup();
#else
  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }
  // Output zero power
  motorsStop();
#endif

  isInit = true;
}

void motorsDeInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

bool motorsTest(void)
{
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    if (motorMap[i]->drvType == BRUSHED)
    {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
      motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsSetRatio(MOTORS[i], 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }

  return isInit;
}

void motorsStop()
{
  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
    motorsSetRatio(MOTORS[i], powerDistributionStopRatio(i));
  }

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
  if (motorMap[0]->drvType == BRUSHLESS)
  {
    motorsBurstDshot();
  }
#endif
}

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static void motorsDshotSetup()
{
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* DMA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // Preparation of common things in DMA setup struct that will be the same every time
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;

  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
#ifndef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
//    motorsDshotOutputSetup(i);
#endif
    NVIC_InitStructure.NVIC_IRQChannel = motorMap[i]->DMA_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MOTORS_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MOTORS_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void motorsDshotOutputSetup(int id)
{
  TIM_OCInitTypeDef TIM_OCInitStructure;

  // Due to complicated ESC startup behavior, it is best to have the DHOT output
  // running when we release the reset so it doesn't enter bootloader mode so
  // that it detects the DHOT signal correct (inverted or none-inverted). So we
  // wait and do the reset first when the stabalizer task is running.
  if (doResetESCs && motorMap[id]->hasPC15ESCReset) 
  {
      GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);
      vTaskDelay(M2T(1));
      GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
      doResetESCs = false;
  }

  TIM_Cmd(motorMap[id]->tim, DISABLE);
  motorMap[id]->tim->ARR = motorMap[id]->timPeriod;
  motorMap[id]->tim->CNT = 0;
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_ITConfig(motorMap[id]->tim, TIM_IT_Update, DISABLE);
  TIM_Cmd(motorMap[id]->tim, ENABLE);

  DMA_InitStructureShare.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = motorMap[id]->DMA_PerifAddr;
  DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dshotDmaBuffer[id];
  DMA_InitStructureShare.DMA_Channel = motorMap[id]->DMA_Channel;
  DMA_Init(motorMap[id]->DMA_stream, &DMA_InitStructureShare);
}

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
static void motorsDshotInputSetup(int id)
{
  dshotState[id] = DSHOT_STATE_INPUT;

  bool timerAllIdle = true;
  for (int i = 0; i < NBR_OF_MOTORS; i++) {
    timerAllIdle &= (dshotState[i] == DSHOT_STATE_IDLE || dshotState[i] == DSHOT_STATE_INPUT);
  }

//  if (timerAllIdle) {
  if (dshotState[0] == DSHOT_STATE_INPUT &&
      dshotState[2] == DSHOT_STATE_INPUT &&
      dshotState[3] == DSHOT_STATE_INPUT) {
    // Special case for M2 that is delayed 100us due to resourse conflict.
    TIM_Cmd(motorMap[1]->tim, DISABLE);
    motorMap[id]->tim->ARR = TIM_CLOCK_HZ / 10000; // 100us max interval
    motorMap[id]->tim->CNT = 0;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(motorMap[1]->tim, TIM_IT_Update, ENABLE);
    TIM_Cmd(motorMap[1]->tim, ENABLE);
  }

  TIM_ICInitTypeDef TIM_ICInitStructure;
  
  TIM_ICInitStructure.TIM_Channel = motorMap[id]->timChannel;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x02;
  TIM_ICInit(motorMap[id]->tim, &TIM_ICInitStructure);

  // Only changing what is different from output setup
  DMA_InitStructureShare.DMA_BufferSize = DSHOT_TELEMETRY_MAX_GCR_EDGES;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructureShare.DMA_PeripheralBaseAddr = motorMap[id]->DMA_PerifAddr;
  DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dshotDmaInputBuffer[id];
  DMA_InitStructureShare.DMA_Channel = motorMap[id]->DMA_Channel;
  DMA_Init(motorMap[id]->DMA_stream, &DMA_InitStructureShare);
  /* Enable TIM DMA Requests*/
  TIM_DMACmd(motorMap[id]->tim, motorMap[id]->TIM_DMASource, ENABLE);
  DMA_ITConfig(motorMap[id]->DMA_stream, DMA_IT_TC, ENABLE);
  /* Enable DMA TIM Stream */
  DMA_Cmd(motorMap[id]->DMA_stream, ENABLE);
}

static uint16_t dshotDecodeTelemetryPacket(const uint32_t *buffer, uint32_t gcrEdges)
{
  // Source: https://github.com/betaflight/betaflight/blob/0b94db5fee44ec5af9f899a229aa329c90cbd8a8/src/platform/common/stm32/pwm_output_dshot_shared.c#L164
  uint32_t value = 0;
  uint32_t oldValue = buffer[0];
  int bits = 0;
  int len;
  for (uint32_t i = 1; i <= gcrEdges; i++) {
      if (i < gcrEdges) {
          int diff = buffer[i] - oldValue;
          if (bits >= 21) {
              break;
          }
          len = (diff + (DSHOT_TELEMETRY_GCR_BIT_PERIOD / 2)) / DSHOT_TELEMETRY_GCR_BIT_PERIOD;
      } else {
          len = 21 - bits;
      }

      value <<= len;
      value |= 1 << (len - 1);
      oldValue = buffer[i];
      bits += len;
  }
  if (bits != 21) {
      return DSHOT_TELEMETRY_INVALID;
  }

  static const uint32_t decode[32] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
      0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

  uint32_t decodedValue = decode[value & 0x1f];
  decodedValue |= decode[(value >> 5) & 0x1f] << 4;
  decodedValue |= decode[(value >> 10) & 0x1f] << 8;
  decodedValue |= decode[(value >> 15) & 0x1f] << 12;

  uint32_t csum = decodedValue;
  csum = csum ^ (csum >> 8); // xor bytes
  csum = csum ^ (csum >> 4); // xor nibbles

  if ((csum & 0xf) != 0xf) {
      return DSHOT_TELEMETRY_INVALID;
  }

  return decodedValue >> 4;
}

static uint16_t dshotDecodeTelemetryERPM(uint16_t valueGCR)
{
  uint16_t period;
  uint16_t rpm;
    
  // eRPM range
  if (valueGCR == 0x0fff) {
    return DSHOT_ERPM_INVALID;
  } else if (valueGCR == 0x0fff) {
      return 0;
  }

  // Convert value to 16 bit period from the GCR telemetry format (eeem mmmm mmmm)
  period = (valueGCR & 0x01ff) << ((valueGCR & 0xfe00) >> 9);
  if (!period) {
      return DSHOT_ERPM_INVALID;
  }

  // Convert period to rpm
  rpm = (uint16_t)((1000000UL * 60 * 2 / period) / (motorNbrOfPoles ));
  return rpm;
}
#endif

static void motorsPrepareDshot(uint32_t id, uint16_t ratio)
{
  uint16_t dshotBits;
  bool dshot_telemetry = false;
  uint16_t dshotRatio;

  ASSERT(id < NBR_OF_MOTORS);

  // Scale 16 -> 11 bits
  dshotRatio = (ratio >> 5);
  // Remove command area of DSHOT
  if (dshotRatio < (DSHOT_MIN_THROTTLE - 1))
  {
    dshotRatio = 0;
  }

  dshotBits = (dshotRatio << 1) | (dshot_telemetry ? 1 : 0);

  // compute checksum
  unsigned cs = 0;
  unsigned csData = dshotBits;

  for (int i = 0; i < 3; i++)
  {
    cs ^=  csData; // xor data by nibbles
    csData >>= 4;
  }

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
    cs = ~cs;
#endif

  cs &= 0xf;
  dshotBits = (dshotBits << 4) | cs;

  for(int i = 0; i < DSHOT_FRAME_SIZE; i++)
  {
    dshotDmaBuffer[id][i] = (dshotBits & 0x8000) ? MOTORS_TIM_VALUE_FOR_1 : MOTORS_TIM_VALUE_FOR_0;
    dshotBits <<= 1;
  }
  dshotDmaBuffer[id][16] = 0; // Set to 0 gives low output afterwards

  // Wait for DMA to be free. Can happen at startup but doesn't seem to wait afterwards.
  while(DMA_GetCmdStatus(motorMap[id]->DMA_stream) != DISABLE)
  {
    dmaWait++;
  }
}

/**
 * Unfortunately the TIM2_CH2 (M1) and TIM2_CH4 (M2) share DMA channel 3 request and can't
 * be used at the same time. Solved by running after each other and TIM2_CH2
 * will be started in DMA1_Stream6_IRQHandler. Thus M2 will have a bit of latency.
 */
void motorsBurstDshot()
{
  for (int i = 0; i < NBR_OF_MOTORS; i++) 
  {
//    DMA_Cmd(motorMap[i]->DMA_stream, DISABLE);
//    TIM_DMACmd(motorMap[i]->tim, motorMap[i]->TIM_DMASource, DISABLE);
  }

  dshotState[0] = DSHOT_STATE_OUTPUT;
  DMA_ClearITPendingBit(motorMap[0]->DMA_stream, motorMap[0]->DMA_ITFlag_TC);
  motorsDshotOutputSetup(0);
  /* Enable TIM DMA Requests M1*/
  TIM_DMACmd(motorMap[0]->tim, motorMap[0]->TIM_DMASource, ENABLE);
  DMA_ITConfig(motorMap[0]->DMA_stream, DMA_IT_TC, ENABLE);
  /* Enable DMA TIM Stream */
  DMA_Cmd(motorMap[0]->DMA_stream, ENABLE);


  // dshotState[1] = DSHOT_STATE_OUTPUT;
  // DMA_ClearITPendingBit(motorMap[1]->DMA_stream, motorMap[1]->DMA_ITFlag_TC);
  // motorsDshotOutputSetup(1);
  // /* Enable TIM DMA Requests M2*/
  // DMA_ITConfig(motorMap[1]->DMA_stream, DMA_IT_TC, ENABLE);
  // // TIM DMA Request and DMA stream for M2 will be started delayed

  // dshotState[2] = DSHOT_STATE_OUTPUT;
  // DMA_ClearITPendingBit(motorMap[2]->DMA_stream, motorMap[2]->DMA_ITFlag_TC);
  // motorsDshotOutputSetup(2);
  // /* Enable TIM DMA Requests M3*/
  // TIM_DMACmd(motorMap[2]->tim, motorMap[2]->TIM_DMASource, ENABLE);
  // DMA_ITConfig(motorMap[2]->DMA_stream, DMA_IT_TC, ENABLE);
  // /* Enable DMA TIM Stream */
  // DMA_Cmd(motorMap[2]->DMA_stream, ENABLE);

  // dshotState[3] = DSHOT_STATE_OUTPUT;
  // DMA_ClearITPendingBit(motorMap[3]->DMA_stream, motorMap[3]->DMA_ITFlag_TC);
  // motorsDshotOutputSetup(3);
  // /* Enable TIM DMA Requests M4*/
  // TIM_DMACmd(motorMap[3]->tim, motorMap[3]->TIM_DMASource, ENABLE);
  // DMA_ITConfig(motorMap[3]->DMA_stream, DMA_IT_TC, ENABLE);
  // /* Enable DMA TIM Stream */
  // DMA_Cmd(motorMap[3]->DMA_stream, ENABLE);
}

#endif

// Ithrust is thrust mapped for 65536 <==> max thrust
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  if (isInit) {
    ASSERT(id < NBR_OF_MOTORS);

    uint16_t ratio = ithrust;

    // Override ratio in case of motorSetEnable
    if (motorSetEnable == 1)
    {
      ratio = motorPowerSet[id];
    }
    else if (motorSetEnable == 2)
    {
      ratio = motorPowerSet[MOTOR_M1];
    }
    else if (motorSetEnable == 3) // for testing the battery compensation
    {
      float b = 0.01f; // 0.2f = Convergence (95%) in ~10 steps = ~20ms
      static float supplyVoltage = 4.2;
      // only update the voltage for the first motor and use the same for the others, as done in stabilizer.c
      if (id == 0) 
      {
        supplyVoltage = supplyVoltage + b * (pmGetBatteryVoltage() - supplyVoltage);
      }
      uint32_t ratioUncapped = motorsCompensateBatteryVoltage(id, motorPowerSet[MOTOR_M1], supplyVoltage);

      // since motor_ratios are 16 bit, the ratio needs to be capped as in the regular code
      if (ratioUncapped > UINT16_MAX)
      {
        ratio = UINT16_MAX;
      }
      else
      {
        ratio = ratioUncapped;
      }
    }

    motor_ratios[id] = ratio;

    if (motorMap[id]->drvType == BRUSHLESS)
    {
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
      // Prepare DSHOT, firing it will be done synchronously with motorsBurstDshot.
      motorsPrepareDshot(id, ratio);
#else
      motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
#endif
    }
    else
    {
      motorMap[id]->setCompare(motorMap[id]->tim, motorsConv16ToBits(ratio));
    }

    if (id == MOTOR_M1)
    {
      uint64_t currTime = usecTimestamp();
      cycleTime = currTime - lastCycleTime;
      lastCycleTime = currTime;
    }
  }
}

void motorsEnablePWM(void)
{
  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);
  }
}

void motorsDisablePWM(void)
{
  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_CtrlPWMOutputs(motorMap[i]->tim, DISABLE);
  }
}

void motorsEnablePassthough(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);

  TIM_CtrlPWMOutputs(motorMap[id]->tim, DISABLE);

  motorsESCSetInput(id);
  motorsESCSetHi(id);
}

void motorsESCSetInput(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  GPIO_PassthroughInput.GPIO_Pin = motorMap[id]->gpioPin;
  GPIO_Init(motorMap[id]->gpioPort, &GPIO_PassthroughInput);
}

void motorsESCSetOutput(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  GPIO_PassthroughOutput.GPIO_Pin = motorMap[id]->gpioPin;
  GPIO_PassthroughOutput.GPIO_OType = motorMap[id]->gpioOType;
  GPIO_Init(motorMap[id]->gpioPort, &GPIO_PassthroughOutput);
}

void motorsESCSetHi(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  GPIO_WriteBit(motorMap[id]->gpioPort, motorMap[id]->gpioPin, Bit_SET);
}

void motorsESCSetLo(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  GPIO_WriteBit(motorMap[id]->gpioPort, motorMap[id]->gpioPin, Bit_RESET);
}

int motorsESCIsHi(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  return GPIO_ReadInputDataBit(motorMap[id]->gpioPort, motorMap[id]->gpioPin) != Bit_RESET;
}

int motorsESCIsLo(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);
  return GPIO_ReadInputDataBit(motorMap[id]->gpioPort, motorMap[id]->gpioPin) == Bit_RESET;
}

uint16_t motorsGetRatio(uint32_t id)
{
  ASSERT(id < NBR_OF_MOTORS);

  return motor_ratios[id];
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  ASSERT(id < NBR_OF_MOTORS);

  if (motorMap[id]->drvType == BRUSHED)
  {
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    if (enable)
    {
      TIM_TimeBaseStructure.TIM_Prescaler = (5 - 1);
      TIM_TimeBaseStructure.TIM_Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
    }
    else
    {
      TIM_TimeBaseStructure.TIM_Period = motorMap[id]->timPeriod;
      TIM_TimeBaseStructure.TIM_Prescaler = motorMap[id]->timPrescaler;
    }

    // Timer configuration
    TIM_TimeBaseInit(motorMap[id]->tim, &TIM_TimeBaseStructure);
    motorMap[id]->setCompare(motorMap[id]->tim, ratio);
  }
}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
  motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  vTaskDelay(M2T(duration_msec));
  motorsBeep(MOTOR_M1, false, frequency, 0);
  motorsBeep(MOTOR_M2, false, frequency, 0);
  motorsBeep(MOTOR_M3, false, frequency, 0);
  motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
  int i = 0;
  uint16_t note;      // Note in hz
  uint16_t duration;  // Duration in ms

  do
  {
    note = notes[i++];
    duration = notes[i++];
    motorsPlayTone(note, duration);
  } while (duration != 0);
}

const MotorHealthTestDef* motorsGetHealthTestSettings(uint32_t id)
{
  if (id >= NBR_OF_MOTORS)
  {
    return &unknownMotorHealthTestSettings;
  }
  else if (motorMap[id]->drvType == BRUSHLESS)
  {
    return &brushlessMotorHealthTestSettings;
  }
  else if (motorMap[id]->drvType == BRUSHED)
  {
    return &brushedMotorHealthTestSettings;
  }
  else
  {
    return &unknownMotorHealthTestSettings;
  }
}

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static void motorsDshotTransferEnded(int id)
{
  dshotState[id] = DSHOT_STATE_IDLE;

  // bool allIdle = true;
  // for (int i = 0; i < NBR_OF_MOTORS; i++) {
  //   allIdle &= (dshotState[i] == DSHOT_STATE_IDLE);
  // }

// //  if (allIdle) {
//   if (dshotState[0] == DSHOT_STATE_IDLE &&
//       dshotState[2] == DSHOT_STATE_IDLE &&
//       dshotState[3] == DSHOT_STATE_IDLE) 
//   {
//     TIM_Cmd(motorMap[1]->tim, DISABLE);
//     motorMap[1]->tim->ARR = motorMap[1]->timPeriod;
//     motorMap[1]->tim->CNT = 0;
//     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//     TIM_ITConfig(motorMap[1]->tim, TIM_IT_Update, DISABLE);
//     TIM_Cmd(motorMap[1]->tim, ENABLE);

//     dshotState[1] = DSHOT_STATE_OUTPUT;
//     /* Enable TIM DMA Requests M1*/
//     TIM_DMACmd(motorMap[1]->tim, motorMap[1]->TIM_DMASource, ENABLE);
//     /* Enable DMA TIM Stream */
//     DMA_Cmd(motorMap[1]->DMA_stream, ENABLE);
//   }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
  uint32_t gcrEdges = DSHOT_TELEMETRY_MAX_GCR_EDGES - motorMap[id]->DMA_stream->NDTR;
  if (gcrEdges > DSHOT_TELEMETRY_MIN_GCR_EDGES) {
    dshotTelemetryPackets[id] = dshotDecodeTelemetryPacket(dshotDmaInputBuffer[id], gcrEdges);
    motorRPMs[id] = dshotDecodeTelemetryERPM(dshotTelemetryPackets[id]);
  }
#endif
}

void __attribute__((used)) DMA1_Stream1_IRQHandler(void)  // M4
{
  TIM_DMACmd(TIM2, TIM_DMA_CC3, DISABLE);
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, DISABLE);
  DMA_Cmd(DMA1_Stream1, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
  if (dshotState[3] == DSHOT_STATE_OUTPUT) {
    motorsDshotInputSetup(3);
  } else {
    motorsDshotTransferEnded(3);
  }
#else
  motorsDshotTransferEnded(3);
#endif
}

void __attribute__((used)) DMA1_Stream5_IRQHandler(void)  // M3
{
  TIM_DMACmd(TIM2, TIM_DMA_CC1, DISABLE);
  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
  DMA_Cmd(DMA1_Stream5, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
  if (dshotState[2] == DSHOT_STATE_OUTPUT) {
    motorsDshotInputSetup(2);
  } else {
    motorsDshotTransferEnded(2);
  }
#else
  motorsDshotTransferEnded(2);
#endif
}

void __attribute__((used)) DMA1_Stream6_IRQHandler(void) // M1
{
  TIM_DMACmd(TIM2, TIM_DMA_CC2, DISABLE);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
//  DMA_Cmd(DMA1_Stream6, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
  if (dshotState[0] == DSHOT_STATE_OUTPUT) {
    motorsDshotInputSetup(0);
  } else {
    motorsDshotTransferEnded(0);
  }
#else
  motorsDshotTransferEnded(0);
#endif
}
// The problematic DMA channel due to request sharing with M4
void __attribute__((used)) DMA1_Stream7_IRQHandler(void)  // M2
{
  TIM_DMACmd(TIM2, TIM_DMA_CC4, DISABLE);
  DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, DISABLE);
  DMA_Cmd(DMA1_Stream7, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
  if (dshotState[1] == DSHOT_STATE_OUTPUT) {
    motorsDshotInputSetup(1);
  } else {
    motorsDshotTransferEnded(1);
  }
#else
  dshotState[1] = DSHOT_STATE_IDLE;
#endif
}

// Used to terminate receiving
void __attribute__((used)) TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_ITConfig(motorMap[1]->tim, TIM_IT_Update, DISABLE);

  /* Enable special case TIM DMA Requests M2*/
  TIM_DMACmd(motorMap[1]->tim, motorMap[1]->TIM_DMASource, ENABLE);
  DMA_ITConfig(motorMap[1]->DMA_stream, DMA_IT_TC, ENABLE);
  /* Enable DMA TIM Stream */
  DMA_Cmd(motorMap[1]->DMA_stream, ENABLE);
#endif
}

// // Used to terminate receiving
// void __attribute__((used)) TIM2_IRQHandler(void)
// {
//   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

// #ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
//   if (dshotState[0] == DSHOT_STATE_INPUT) {
//     TIM_DMACmd(TIM2, TIM_DMA_CC2, DISABLE);
//     DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
//     DMA_Cmd(DMA1_Stream6, DISABLE);
//     DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
    
//     motorsDshotTransferEnded(0);
//   }

//   if (dshotState[1] == DSHOT_STATE_INPUT) {
//     TIM_DMACmd(TIM2, TIM_DMA_CC4, DISABLE);
//     DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, DISABLE);
//     DMA_Cmd(DMA1_Stream7, DISABLE);
//     DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);

//     dshotState[1] = DSHOT_STATE_IDLE;
//   }
  
//   if (dshotState[2] == DSHOT_STATE_INPUT) {
//     TIM_DMACmd(TIM2, TIM_DMA_CC1, DISABLE);
//     DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
//     DMA_Cmd(DMA1_Stream5, DISABLE);
//     DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
    
//     motorsDshotTransferEnded(2);
//   }

//   if (dshotState[3] == DSHOT_STATE_INPUT) {
//     TIM_DMACmd(TIM2, TIM_DMA_CC3, DISABLE);
//     DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, DISABLE);
//     DMA_Cmd(DMA1_Stream1, DISABLE);
//     DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
    
//     motorsDshotTransferEnded(3);
//   }
// #endif
// }

//#endif

/**
 * Override power distribution to motors.
 */
PARAM_GROUP_START(motorPowerSet)

/**
 * @brief Nonzero to override controller with set values.
 * 1 to hand PWM right to the motors, 2 to hand m1 to all motors, 3 to hand m1 to all motors and activate battery compensation.
 */
PARAM_ADD_CORE(PARAM_UINT8, enable, &motorSetEnable)

/**
 * @brief motor power for m1: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m1, &motorPowerSet[0])

/**
 * @brief motor power for m2: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m2, &motorPowerSet[1])

/**
 * @brief motor power for m3: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m3, &motorPowerSet[2])

/**
 * @brief motor power for m4: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m4, &motorPowerSet[3])

PARAM_GROUP_STOP(motorPowerSet)

/**
 * Motor output related log variables.
 */
LOG_GROUP_START(motor)
/**
 * @brief Motor power (PWM value) for M1 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m1, &motor_ratios[MOTOR_M1])
/**
 * @brief Motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m2, &motor_ratios[MOTOR_M2])
/**
 * @brief Motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m3, &motor_ratios[MOTOR_M3])
/**
 * @brief Motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT16, m4, &motor_ratios[MOTOR_M4])

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL
/**
 * @brief Motor RPM telemetry for M1. UINT16_MAX is no value.
 */
LOG_ADD(LOG_UINT16, m1_rpm, &motorRPMs[MOTOR_M1])
/**
 * @brief Motor RPM telemetry for M2. UINT16_MAX is no value.
 */
LOG_ADD(LOG_UINT16, m2_rpm, &motorRPMs[MOTOR_M2])
/**
 * @brief Motor RPM telemetry for M3. UINT16_MAX is no value.
 */
LOG_ADD(LOG_UINT16, m3_rpm, &motorRPMs[MOTOR_M3])
/**
 * @brief Motor RPM telemetry for M4. UINT16_MAX is no value.
 */
LOG_ADD(LOG_UINT16, m4_rpm, &motorRPMs[MOTOR_M4])
#endif

LOG_GROUP_STOP(motor)
