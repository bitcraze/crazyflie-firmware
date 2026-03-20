/*
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
 * system.h - Top level module header file
 */

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the system module.
 * 
 * Performs all necessary initialization for the system module,
 * setting up required hardware and software resources.
 */
void systemInit(void);

/**
 * @brief Test the system module.
 * 
 * Runs self-tests on the system module to verify correct initialization
 * and functionality.
 * 
 * @return true if the system test passes, false otherwise.
 */
bool systemTest(void);

/**
 * @brief Launch the system.
 * 
 * Triggers the system launch sequence, transitioning the system
 * from an initialized state to an operational state.
 */
void systemLaunch(void);

/**
 * @brief Start the system.
 * 
 * Starts the system's main operation, enabling all active modules
 * and tasks.
 */
void systemStart();

/**
 * @brief Wait for the system to start.
 * 
 * Blocks the calling task until the system has fully started
 * and is ready for operation.
 */
void systemWaitStart(void);

/**
 * @brief Request a system shutdown.
 * 
 * Initiates a shutdown of the entire system.
 */
void systemRequestShutdown();

/**
 * @brief Request a shutdown of the STM32 microcontroller.
 * 
 * Initiates a shutdown of STM32 and decks (VCC & VCOM).
 */
void systemRequestShutdownSTM();

/**
 * @brief Request the NRF firmware version.
 * 
 * Sends a request to the NRF processor to retrieve its
 * current firmware version information.
 */
void systemRequestNRFVersion();

/**
 * @brief Send a radio ready notification.
 * 
 * Notifies the system (and the NRF processor via syslink) that
 * the radio subsystem is initialized and ready for communication.
 */
void systemSendRadioReady();

/**
 * @brief Handle incoming syslink messages.
 * 
 * Processes received syslink packets from the NRF processor,
 * dispatching them to the appropriate system handlers.
 */
void systemSyslinkReceive();

#endif //__SYSTEM_H__
