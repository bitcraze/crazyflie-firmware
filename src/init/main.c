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
 */

 /**
 * @file main.c 
 * @brief Contains the main function that initializes all tasks and starts the scheduler.
 *
 * @defgroup modules
 * @defgroup decks
 * @defgroup system
 *
 * @ingroup system
 */

/** @mainpage Crazyflie firmware documentation
 *
 * The following documentation was automatically generated by doxygen from the firmware's
 * C-code. Please check out the other sources of documentation as well:
 *
 *  [Documentation start page](https://www.bitcraze.io/documentation/start/)
 *
 *  [Firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
 *
 *  [Other components of the ecosystem](https://www.bitcraze.io/documentation/repository/)
 *
 * 
 *
 * Reader's guide to the firmware
 * ------------------------------
 *
 * If you are completely new to the firmware, we recommend you take a look at the following
 * main parts of the system first: 
 *
 *      systemTask()
 *      stabilizerTask()
 *      
 * The guiding principle is that systemTask() calls initializers that create further RTOS tasks
 * such as the stabilizerTask(), sensors, radio communication etc. Once all tasks are created, 
 * systemTask calls all routines for self-test. If tests pass, systemTask() starts the 
 * system by giving the semaphore canStartMutex - unlocking all tasks.  
 *
 * If you are new to embedded multitasking with FreeRTOS, check out a book such as
 * "Hands-On RTOS with Microcontrollers" by B Amos.
 *
 * Building applications on top of the firmware
 * --------------------------------------------
 * 
 * The easiest way to extend the firmware with your own applications or to make an autonomous
 * robot is the app layer that allows to separate your control logic from the lower levels of
 * the system. Check out appInit() and appTask() and the [documentation on the app layer here]
 * (https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/).
 *
 *
 * The path from sensor aquisition to the Python running on your notebook
 * ----------------------------------------------------------------------
 *
 * If you want to follow the flow of data from sensor readings all the way to your desktop or
 * notebook computer, you could use the multiranger deck as an example.
 *
 * Check out mrTask() in multiranger.c -- this is the RTOS task regularly querying the hardware driver
 * for updated raw sensor readings. mrTask() uses rangeSet() in range.c to make the readings available.
 * range.c also defines the log group 'range' into which the readings are copied and hence made
 * available for the [logging system](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/). 
 * As soon as a variable is logged and registered in the logging systems' table of contents, the 
 * logTask() continously sends its content via crazyradio. 
 *
 *
 */

/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
#include "platform.h"
#include "system.h"
#include "usec_time.h"

#include "led.h"

/* ST includes */
#include "stm32fxxx.h"

/**
 * Check hardware compatibility and launch the system
 *
 * Call systemLaunch() and start the RTOS scheduler.
 * If init fails, play a specific LED sequence.
 * 
 * @see platformInit()
 * @see systemLaunch()
 *
 */
int main() 
{
  //Initialize the platform.
  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while(1);
  }

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //TODO: Move to platform launch failed
  ledInit();
  ledSet(0, 1);
  ledSet(1, 1);

  //Should never reach this point!
  while(1);

  return 0;
}

