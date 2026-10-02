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
 * comm.c - High level communication module
 */

#include <stdbool.h>

#include "autoconf.h"
#include "config.h"

#include "crtp.h"
#include "console.h"
#include "crtpservice.h"
#include "param_task.h"
#include "log.h"
#include "platformservice.h"
#ifdef CONFIG_PLATFORM_SIM
#include "udplink_sim.h"
#else
#include "eskylink.h"
#include "uart_syslink.h"
#include "radiolink.h"
#include "usblink.h"
#include "syslink.h"
#include "crtp_localization_service.h"
#endif

static bool isInit;

void commInit(void)
{
  if (isInit)
    return;

#ifdef CONFIG_PLATFORM_SIM
  /* Simmyflie: CRTP is carried over UDP in place of the radio. */
  udplinkInit();
#else
  uartslkInit();
  radiolinkInit();
#endif

  /* These functions are moved to be initialized early so
   * that DEBUG_PRINT can be used early */
  // crtpInit();
  // consoleInit();

#ifdef CONFIG_PLATFORM_SIM
  crtpSetLink(udplinkGetLink());
#else
  crtpSetLink(radiolinkGetLink());
#endif

  crtpserviceInit();
  platformserviceInit();
  logInit();
  paramInit();
#ifndef CONFIG_PLATFORM_SIM
  /* Not built for the Simmyflie */
  locSrvInit();
#endif

  //setup CRTP communication channel
  //TODO: check for USB first and prefer USB over radio
  //if (usbTest())
  //  crtpSetLink(usbGetLink);
  //else if(radiolinkTest())
  //  crtpSetLink(radiolinkGetLink());

  isInit = true;
}

bool commTest(void)
{
  bool pass=isInit;

#ifndef CONFIG_PLATFORM_SIM
  pass &= radiolinkTest();
#endif
  pass &= crtpTest();
  pass &= crtpserviceTest();
  pass &= platformserviceTest();
  pass &= consoleTest();
  pass &= paramTest();

  return pass;
}

