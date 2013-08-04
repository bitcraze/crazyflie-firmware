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
 * nvic.c - Contains all Cortex-M3 processor exceptions handlers
 */
#include "exti.h"
#include "adc.h"
#include "led.h"
#include "uart.h"
#include "i2croutines.h"
#include "i2cdev.h"

#define DONT_DISCARD __attribute__((used))

void nvicInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

#ifdef NVIC_NOT_USED_BY_FREERTOS
/**
 * @brief  This function handles SysTick Handler.
 */
void DONT_DISCARD SysTick_Handler(void)
{
}

/**
  * @brief  This function handles SVCall exception.
  */
void DONT_DISCARD SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void DONT_DISCARD PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles NMI exception.
  */
void DONT_DISCARD NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void DONT_DISCARD HardFault_Handler(void)
{
  //http://www.st.com/mcu/forums-cat-6778-23.html
  //****************************************************
  //To test this application, you can use this snippet anywhere:
  // //Let's crash the MCU!
  // asm (" MOVS r0, #1 \n"
  // " LDM r0,{r1-r2} \n"
  // " BX LR; \n");
  asm( "TST LR, #4 \n"
  "ITE EQ \n"
  "MRSEQ R0, MSP \n"
  "MRSNE R0, PSP \n"
  "B printHardFault");
}

void DONT_DISCARD printHardFault(uint32_t* hardfaultArgs)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfaultArgs[0]);
  stacked_r1 = ((unsigned long) hardfaultArgs[1]);
  stacked_r2 = ((unsigned long) hardfaultArgs[2]);
  stacked_r3 = ((unsigned long) hardfaultArgs[3]);

  stacked_r12 = ((unsigned long) hardfaultArgs[4]);
  stacked_lr = ((unsigned long) hardfaultArgs[5]);
  stacked_pc = ((unsigned long) hardfaultArgs[6]);
  stacked_psr = ((unsigned long) hardfaultArgs[7]);

  uartPrintf("[Hard fault handler]\n");
  uartPrintf("R0 = %x\n", stacked_r0);
  uartPrintf("R1 = %x\n", stacked_r1);
  uartPrintf("R2 = %x\n", stacked_r2);
  uartPrintf("R3 = %x\n", stacked_r3);
  uartPrintf("R12 = %x\n", stacked_r12);
  uartPrintf("LR = %x\n", stacked_lr);
  uartPrintf("PC = %x\n", stacked_pc);
  uartPrintf("PSR = %x\n", stacked_psr);
  uartPrintf("BFAR = %x\n", (*((volatile unsigned int *)(0xE000ED38))));
  uartPrintf("CFSR = %x\n", (*((volatile unsigned int *)(0xE000ED28))));
  uartPrintf("HFSR = %x\n", (*((volatile unsigned int *)(0xE000ED2C))));
  uartPrintf("DFSR = %x\n", (*((volatile unsigned int *)(0xE000ED30))));
  uartPrintf("AFSR = %x\n", (*((volatile unsigned int *)(0xE000ED3C))));

  while (1)
  {}
}
/**
 * @brief  This function handles Memory Manage exception.
 */
void DONT_DISCARD MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void DONT_DISCARD BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void DONT_DISCARD UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void DONT_DISCARD DebugMon_Handler(void)
{
}

void DONT_DISCARD DMA1_Channel1_IRQHandler(void)
{
  adcInterruptHandler();
}

void DONT_DISCARD DMA1_Channel2_IRQHandler(void)
{
#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA)
  uartDmaIsr();
#endif
}

void DONT_DISCARD DMA1_Channel4_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

void DONT_DISCARD DMA1_Channel5_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

void DONT_DISCARD DMA1_Channel6_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}

void DONT_DISCARD DMA1_Channel7_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}


void DONT_DISCARD EXTI9_5_IRQHandler(void)
{
  extiInterruptHandler();
}

void DONT_DISCARD USART3_IRQHandler(void)
{
  uartIsr();
}

void DONT_DISCARD TIM1_UP_IRQHandler(void)
{
  extern uint32_t usecTimerHighCount;

  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

  __sync_fetch_and_add(&usecTimerHighCount, 1);
}

void DONT_DISCARD I2C1_EV_IRQHandler(void)
{
  i2cInterruptHandlerI2c1();
}

void DONT_DISCARD I2C1_ER_IRQHandler(void)
{
  i2cErrorInterruptHandlerI2c1();
}

void DONT_DISCARD I2C2_EV_IRQHandler(void)
{

}

void DONT_DISCARD I2C2_ER_IRQHandler(void)
{
  I2C_ClearFlag(I2C2, 0x1000FFFF);
}

