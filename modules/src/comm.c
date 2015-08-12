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

#include "config.h"

#include "nrf24link.h"
#include "crtp.h"
#include "console.h"
#include "crtpservice.h"
#include "param.h"
#include "log.h"
#include "eskylink.h"
#include "radiolink.h"
#include "nrf24link.h"
#include "usblink.h"
#include "platformservice.h"
#include "syslink.h"

static bool isInit;

void commInit(void)
{
  if (isInit)
    return;
#ifdef PLATFORM_CF1
  #ifdef USE_ESKYLINK
    eskylinkInit();
  #else
    nrf24linkInit();
  #endif
#else
  usblinkInit();
  radiolinkInit();
#endif

  /* These functions are moved to be initialized early so
   * that DEBUG_PRINT can be used early */
  // crtpInit();
  // consoleInit();

#ifdef USE_RADIOLINK_CRTP
  crtpSetLink(radiolinkGetLink());
#elif defined(USE_ESKYLINK)
  crtpSetLink(eskylinkGetLink());
#else
  crtpSetLink(nrf24linkGetLink());
#endif

  crtpserviceInit();
#ifdef PLATFORM_CF2
  platformserviceInit();
#endif
  logInit();
  paramInit();
  
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
  
#ifdef PLATFORM_CF1
  #ifdef USE_ESKYLINK
    pass &= eskylinkTest();
  #else
    pass &= nrf24linkTest();
  #endif
#else
  pass &= radiolinkTest();
#endif
  
  pass &= crtpTest();
  pass &= crtpserviceTest();
#ifdef PLATFORM_CF2
  pass &= platformserviceTest();
#endif
  pass &= consoleTest();
  pass &= paramTest();
  
  return pass;
}

