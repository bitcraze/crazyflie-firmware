/* FreeRTOSConfig.h - Shadow header for the Simmyflie.
 *
 * FreeRTOS configuration for the Posix port, which the sim build uses
 * instead of the ARM_CM4F port. The tick rate and the M2T()/T2M() macros
 * match the mainline FreeRTOSConfig.h, so timing in mainline code is
 * unchanged. Interrupt priorities and the run-time statistics hooks of the
 * mainline file are left out; the Posix port defines what it needs. The
 * heap is the host's malloc() (heap_3).
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                   1
#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    0
#define configCPU_CLOCK_HZ                     ( 100000000UL )
#define configTICK_RATE_HZ                     ( (TickType_t) 1000 )
#define configMAX_PRIORITIES                   ( 6 )
#define configMINIMAL_STACK_SIZE               ( 4096 )
#define configTOTAL_HEAP_SIZE                  ( 0 ) /* unused: heap_3 wraps malloc/free */
#define configMAX_TASK_NAME_LEN                ( 16 )
#define configUSE_16_BIT_TICKS                 0
#define configIDLE_SHOULD_YIELD                1
#define configUSE_MUTEXES                      1
/* log.c builds its periodic log blocks on FreeRTOS software timers. */
#define configUSE_TIMERS                       1
#define configTIMER_TASK_PRIORITY              1
#define configTIMER_QUEUE_LENGTH               20
#define configTIMER_TASK_STACK_DEPTH           (configMINIMAL_STACK_SIZE * 4)
#define configCHECK_FOR_STACK_OVERFLOW         0
#define configUSE_MALLOC_FAILED_HOOK           1
#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        ( 2 )
#define configQUEUE_REGISTRY_SIZE              10
#define configUSE_APPLICATION_TASK_TAG         1
/* sysload.c (built as is) lists the tasks with uxTaskGetSystemState() when
 * the system.taskDump param is set. */
#define configUSE_TRACE_FACILITY               1
#define configSUPPORT_STATIC_ALLOCATION        1
#define configSUPPORT_DYNAMIC_ALLOCATION       1

// Same priorities as mainline firmware
#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
#define TASK_PROXIMITY_ID_NBR   6

#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskCleanUpResources          1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_uxTaskGetStackHighWaterMark    1
#define INCLUDE_xTaskGetIdleTaskHandle         1
#define INCLUDE_xTimerPendFunctionCall         0

#define configKERNEL_INTERRUPT_PRIORITY        255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   191

/* As in mainline: some sources (crtp_commander_generic.c,
 * crtp_commander_rpyt.c) get ASSERT() through FreeRTOS.h. */
#include "cfassert.h"
/* As in mainline: crtpservice.c gets its task settings from config.h this
 * way. */
#include "config.h"
#define configASSERT( x )  if( ( x ) == 0 ) assertFail(#x, __FILE__, __LINE__ )

/* Milliseconds to OS ticks, as in mainline. */
#define M2T(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X))
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)

#endif /* FREERTOS_CONFIG_H */
