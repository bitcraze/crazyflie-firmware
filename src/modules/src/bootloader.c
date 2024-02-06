/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) Bitcraze AB
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
 * @file bootloader.c
 * Functions to handle transitioning from the firmware to bootloader (DFU) mode on startup
 *
 */

#include "bootloader.h"

// bootloader code based from micropython machine_bootloader function

// STM32H7 has ECC and writes to RAM must be 64-bit so they are fully committed
// to actual SRAM before a system reset occurs.
#define BL_STATE_PTR                ((uint64_t *) SRAM2_BASE)  //start of 16kb SRAM bank in stm32f405
#define BL_STATE_KEY                (0x5a5) //arbitrary bit pattern used as a marker
#define BL_STATE_KEY_MASK           (0xfff)
#define BL_STATE_KEY_SHIFT          (32)
#define BL_STATE_INVALID            (0)
#define BL_STATE_VALID(reg, addr)   ((uint64_t)(reg) | ((uint64_t)((addr) | BL_STATE_KEY)) << BL_STATE_KEY_SHIFT)
#define BL_STATE_GET_REG(s)         ((s) & 0xffffffff)
#define BL_STATE_GET_KEY(s)         (((s) >> BL_STATE_KEY_SHIFT) & BL_STATE_KEY_MASK)
#define BL_STATE_GET_ADDR(s)        (((s) >> BL_STATE_KEY_SHIFT) & ~BL_STATE_KEY_MASK)


/**
 * @brief Branch directly to the bootloader address, setting the 
 * stack pointer and destination address first.
 * Based from the micropython machine_bootloader function.
 * 
 * @param r0 The register to utilize
 * @param bl_addr The bootloader address to jump to
 */
static void branch_to_bootloader(uint32_t r0, uint32_t bl_addr){
    __asm volatile (
        "ldr r2, [r1, #0]\n"    // get address of stack pointer
        "msr msp, r2\n"         // get stack pointer
        "ldr r2, [r1, #4]\n"    // get address of destination
        "bx r2\n"               // branch to bootloader
        );
    //unreachable code
    while(1);
}

void check_enter_bootloader(){
    uint64_t bl_state = *BL_STATE_PTR;
    //set to invalid for next boot
    *BL_STATE_PTR = BL_STATE_INVALID;

    if(BL_STATE_GET_KEY(bl_state) == BL_STATE_KEY && (RCC->CSR & RCC_CSR_SFTRSTF)){
        //if botloader data valid and was just reset with NVIC_SystemReset

        //remap memory to system flash
        SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SystemFlash);
        branch_to_bootloader(BL_STATE_GET_REG(bl_state), BL_STATE_GET_ADDR(bl_state));
    }
}

void enter_bootloader(uint32_t r0, uint32_t bl_addr){
    //set bootloader state values    
    *BL_STATE_PTR = BL_STATE_VALID(r0, bl_addr);
    
    NVIC_SystemReset();
}