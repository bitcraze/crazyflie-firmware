
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
 */
#define DEBUG_MODULE "MTR-DRV"

#include <stdbool.h>

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
static uint32_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static DMA_InitTypeDef DMA_InitStructureShare;
// Memory buffer for DSHOT bits
static uint32_t dshotDmaBuffer[NBR_OF_MOTORS][DSHOT_DMA_BUFFER_SIZE];
static void motorsDshotDMASetup();
static volatile uint32_t dmaWait;
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

// We have data that maps PWM to thrust at different supply voltage levels.
// However, it is not the PWM that drives the motors but the voltage and
// amps (= power). With the PWM it is possible to simulate different
// voltage levels. The assumption is that the voltage used will be an
// procentage of the supply voltage, we assume that 50% PWM will result in
// 50% voltage.
//
//  Thrust (g)    Supply Voltage    PWM (%)     Voltage needed
//  0.0           4.01              0           0
//  1.6           3.98              6.25        0.24875
//  4.8           3.95              12.25       0.49375
//  7.9           3.82              18.75       0.735
//  10.9          3.88              25          0.97
//  13.9          3.84              31.25       1.2
//  17.3          3.80              37.5        1.425
//  21.0          3.76              43.25       1.6262
//  24.4          3.71              50          1.855
//  28.6          3.67              56.25       2.06438
//  32.8          3.65              62.5        2.28125
//  37.3          3.62              68.75       2.48875
//  41.7          3.56              75          2.67
//  46.0          3.48              81.25       2.8275
//  51.9          3.40              87.5        2.975
//  57.9          3.30              93.75       3.09375
//
// To get Voltage needed from wanted thrust we can get the quadratic
// polyfit coefficients using GNU octave:
//
// thrust = [0.0 1.6 4.8 7.9 10.9 13.9 17.3 21.0 ...
//           24.4 28.6 32.8 37.3 41.7 46.0 51.9 57.9]
//
// volts  = [0.0 0.24875 0.49375 0.735 0.97 1.2 1.425 1.6262 1.855 ...
//           2.064375 2.28125 2.48875 2.67 2.8275 2.975 3.09375]
//
// p = polyfit(thrust, volts, 2)
//
// => p = -0.00062390   0.08835522   0.06865956
//
// We will not use the constant term, since we want zero thrust to equal
// zero PWM.
//
// And to get the PWM as a percentage we would need to divide the
// Voltage needed with the Supply voltage.
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage)
{
  #ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
  ASSERT(id < NBR_OF_MOTORS);

  if (motorMap[id]->drvType == BRUSHED)
  {
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
      return iThrust;
    }

    float thrust = (iThrust / 65536.0f) * 60;
    float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
    float ratio = volts / supplyVoltage;
    return UINT16_MAX * ratio;
  }
  #endif

  return iThrust;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
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

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
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

    // Configure the GPIO for CF-BL ESC RST
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // Hold reset for all CF-BL ESC:s by pulling low.
    GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET);

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);
  }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
  motorsDshotDMASetup();
#endif
  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }

  isInit = true;

  // Output zero power
  motorsStop();
  // Release reset for all CF-BL ESC:s after motor signal is activated
  GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);

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
static void motorsDshotDMASetup()
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* DMA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // Preparation of common things in DMA setup struct
  DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructureShare.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE;
  DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructureShare.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
  DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;

  for (int i = 0; i < NBR_OF_MOTORS; i++)
  {
    DMA_InitStructureShare.DMA_PeripheralBaseAddr = motorMap[i]->DMA_PerifAddr;
    DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dshotDmaBuffer[i];
    DMA_InitStructureShare.DMA_Channel = motorMap[i]->DMA_Channel;
    DMA_Init(motorMap[i]->DMA_stream, &DMA_InitStructureShare);

    NVIC_InitStructure.NVIC_IRQChannel = motorMap[i]->DMA_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MOTORS_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  }
}
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

    motorMap[0]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    motorMap[1]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M1*/
    TIM_DMACmd(motorMap[0]->tim, motorMap[0]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[0]->DMA_stream, DMA_IT_TC, ENABLE);
    DMA_ITConfig(motorMap[1]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[0]->DMA_stream, ENABLE);

    motorMap[2]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M3*/
    TIM_DMACmd(motorMap[2]->tim, motorMap[2]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[2]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[2]->DMA_stream, ENABLE);

    motorMap[3]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M4*/
    TIM_DMACmd(motorMap[3]->tim, motorMap[3]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[3]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[3]->DMA_stream, ENABLE);
}
#endif


// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  if (isInit) {
    ASSERT(id < NBR_OF_MOTORS);

    uint16_t ratio = ithrust;

    // Override ratio in case of motorSetEnable
    if (motorSetEnable == 2)
    {
      ratio = motorPowerSet[MOTOR_M1];
    }
    else if (motorSetEnable == 1)
    {
      ratio = motorPowerSet[id];
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

int motorsGetRatio(uint32_t id)
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
void __attribute__((used)) DMA1_Stream1_IRQHandler(void)  // M4
{
  TIM_DMACmd(TIM2, TIM_DMA_CC3, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, DISABLE);
}
void __attribute__((used)) DMA1_Stream5_IRQHandler(void)  // M3
{
  TIM_DMACmd(TIM2, TIM_DMA_CC1, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
}
void __attribute__((used)) DMA1_Stream6_IRQHandler(void) // M1
{
  TIM_DMACmd(TIM2, TIM_DMA_CC2, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
  /* Enable TIM DMA Requests M2*/
  TIM_DMACmd(motorMap[1]->tim, motorMap[1]->TIM_DMASource, ENABLE);
  /* Enable DMA TIM Stream */
  DMA_Cmd(motorMap[1]->DMA_stream, ENABLE);
}
void __attribute__((used)) DMA1_Stream7_IRQHandler(void)  // M2
{
  TIM_DMACmd(TIM2, TIM_DMA_CC4, DISABLE);
  DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
  DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, DISABLE);
}
#endif


/**
 * Override power distribution to motors.
 */
PARAM_GROUP_START(motorPowerSet)

/**
 * @brief Nonzero to override controller with set values
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
LOG_ADD_CORE(LOG_UINT32, m1, &motor_ratios[MOTOR_M1])
/**
 * @brief Motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m2, &motor_ratios[MOTOR_M2])
/**
 * @brief Motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m3, &motor_ratios[MOTOR_M3])
/**
 * @brief Motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m4, &motor_ratios[MOTOR_M4])
LOG_GROUP_STOP(motor)
