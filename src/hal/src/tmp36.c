#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "tmp36.h"
#include "tmp36_driver.h"
#include "system.h"
#include "param.h"
#include "log.h"

#include "stm32fxxx.h"

/* Flag indicating if the tmp36Init() function has been called or not. */
static bool isInit = false;

/* Measurement variables */
float tempC=0;
float raw_data=0;
float raw_data_avg=0;

/* The most recent samples in chronological order. Must be initialized before use. */
static float tmp36SWin[tmp36_SWIN_SIZE];

#if defined(tmp36_ENABLED)

#if defined(tmp36_LOG_ENABLED)
/* Define a log group. */
LOG_GROUP_START(tmp36)
LOG_ADD(LOG_FLOAT, temperature-celcius, &tempC)
LOG_GROUP_STOP(tmp36)
#endif

/**
 * This function adds a temperature measurement to the sliding window, discarding the oldest sample.
 * After having added the new sample, a new average value of the samples is calculated and returned.
 *
 * @param temperature The new sample to add to the sliding window.
 *
 * @return The new average value of the samples in the sliding window (after adding the new sample).
 */
static float tmp36SWinAdd(float raw_data)
{
  /* Discard oldest sample, move remaining samples one slot to the left. */
  memmove(&tmp36SWin[0], &tmp36SWin[1], (tmp36_SWIN_SIZE - 1) * sizeof(float));

  /* Add the new sample in the last (right-most) slot. */
  tmp36SWin[tmp36_SWIN_SIZE - 1] = raw_data;

  /**
   * Calculate the new average raw_data. Sum all the samples into a float,
   so that we only do a single division at the end.
   */
  float raw_dataNewAvg = 0;
  uint8_t n;
  for (n = 0; n < tmp36_SWIN_SIZE; n++) {
    raw_dataNewAvg += tmp36SWin[n];
  }
  raw_dataNewAvg = raw_dataNewAvg / tmp36_SWIN_SIZE;

  return (float)raw_dataNewAvg;
}

/*
 *This function converts raw data (read from the pin) to temperature in celcius
 *@param data ->raw data read from pin
 *@return the temperature in celcius
 */
float data2C(float data)
{
    //Using V_ref from Vcc of crazyflie : 3000 mV
    //analog in crazyflie is 12 bit (4096)
    float voltage_mv= data*1.0*3000.0/4096.0;
    //tmp36 : 750 mV at 25C, slope :10mV/C
    float temp_c=voltage_mv/10 -50;

    return temp_c;
}
//////////////////////////////////////////////////////////////////////////////
/**
 * Temperature task (tmp36Task) running at tmp36_TASK_FREQ Hz.
 * @param param Currently unused.
 */
static void tmp36Task(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_tmp36_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(tmp36_TASK_FREQ));

    #if defined(tmp36_driver_ENABLED)
    /* Read data the tmp36 sensor. */
    raw_data = raw_data_read(tmp36_GPIO);
    #endif

    /* Get the latest average value calculated. */
    raw_data_avg = tmp36SWinAdd(raw_data);
    tempC=data2C(raw_data_avg);
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
/**
 * Initialization of the tmp36 task.
 */
void tmp36Init(void)
{
  if(isInit)
    return;

  /* Initialise the sliding window to zero. */
  memset(&tmp36SWin, 0, sizeof(float)*tmp36_SWIN_SIZE);

    #if defined(tmp36_ENABLED)
    /* Only start the task if the tmp36 subsystem is enabled in conf.h */
    xTaskCreate(tmp36Task, tmp36_TASK_NAME,tmp36_TASK_STACKSIZE, NULL, tmp36_TASK_PRI, NULL);
    #endif

  isInit = true;
}
