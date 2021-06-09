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
#include "led.h"
#include "motors.h"
#include "cfassert.h"
#include "usb_dcd_int.h"
#include "usb_core.h"

#include "uart1.h"
#define UART_PRINT    uart1Printf

#define DONT_DISCARD __attribute__((used))

void nvicInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

/**
 * @brief  This function handles SysTick Handler.
 */
extern void tickFreeRTOS(void);

void DONT_DISCARD SysTick_Handler(void)
{
    tickFreeRTOS();
}

#ifdef NVIC_NOT_USED_BY_FREERTOS

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
* @brief  STM32_USBF_OTG_ISR_Handler
*         handles all USB Interrupts
* @param  pdev: device instance
* @retval status
*/

void  __attribute__((used)) OTG_FS_IRQHandler(void)
{
  extern USB_OTG_CORE_HANDLE USB_OTG_dev;

  USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

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


  UART_PRINT("[Hard fault handler]\n");
  UART_PRINT("R0 = %x\n", stacked_r0);
  UART_PRINT("R1 = %x\n", stacked_r1);
  UART_PRINT("R2 = %x\n", stacked_r2);
  UART_PRINT("R3 = %x\n", stacked_r3);
  UART_PRINT("R12 = %x\n", stacked_r12);
  UART_PRINT("LR = %x\n", stacked_lr);
  UART_PRINT("PC = %x\n", stacked_pc);
  UART_PRINT("PSR = %x\n", stacked_psr);
  UART_PRINT("BFAR = %x\n", (*((volatile unsigned int *)(0xE000ED38))));
  UART_PRINT("CFSR = %x\n", (*((volatile unsigned int *)(0xE000ED28))));
  UART_PRINT("HFSR = %x\n", (*((volatile unsigned int *)(0xE000ED2C))));
  UART_PRINT("DFSR = %x\n", (*((volatile unsigned int *)(0xE000ED30))));
  UART_PRINT("AFSR = %x\n", (*((volatile unsigned int *)(0xE000ED3C))));

  motorsStop();
  ledShowFaultPattern();

  storeAssertHardfaultData(
    stacked_r0,
    stacked_r1,
    stacked_r2,
    stacked_r3,
    stacked_r12,
    stacked_lr,
    stacked_pc,
    stacked_psr);
  while (1)
  {}
}
/**
 * @brief  This function handles Memory Manage exception.
 */
void DONT_DISCARD MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  ledShowFaultPattern();
  motorsStop();

  storeAssertTextData("MemManage");
  while (1)
  {}
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void DONT_DISCARD BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  motorsStop();
  ledShowFaultPattern();

  storeAssertTextData("BusFault");
  while (1)
  {}
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void DONT_DISCARD UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  motorsStop();
  ledShowFaultPattern();

  storeAssertTextData("UsageFault");
  while (1)
  {}
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void DONT_DISCARD DebugMon_Handler(void)
{
}
