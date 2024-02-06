/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <cassert>
#include <string>

extern "C"
{
  #include "app.h"

  #include "FreeRTOS.h"
  #include "task.h"

  #include "debug.h"
}

#define DEBUG_MODULE "HELLOWORLD"

class MyClass {
  public:
    int myNum;
    std::string myString;
};

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  MyClass *cl = new MyClass();
  DEBUG_PRINT("MyClass has a num: %d\n", cl->myNum);

  /* make sure that the assertion is not simple enough to be optimized away
   * by the compiler */
  assert(cl->myNum + cl->myString.size() == 0);

  while(1) {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Hello World!\n");
  }
}
