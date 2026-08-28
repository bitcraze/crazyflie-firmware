/* FreeRTOSConfig.h for CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * Deliberately standalone -- does not include the mainline src/config/config.h
 * chain (interrupt priorities, NVIC-oriented run-time-stats macros, etc. that
 * only make sense on the real hardware). Settings below match the Phase 0
 * drift-spike prototype (dev/phase0-drift-spike/ in the simulation_model
 * project), which validated the FreeRTOS POSIX port's SIGALRM/setitimer tick
 * against the <=0.1 ms/s real-time drift requirement.
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
#define configUSE_TIMERS                       0
#define configCHECK_FOR_STACK_OVERFLOW         0
#define configUSE_MALLOC_FAILED_HOOK           1
#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        ( 2 )
#define configQUEUE_REGISTRY_SIZE              10
#define configUSE_APPLICATION_TASK_TAG         0
#define configSUPPORT_STATIC_ALLOCATION        0
#define configSUPPORT_DYNAMIC_ALLOCATION       1

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

#include <stdio.h>
#include <stdlib.h>
#define configASSERT( x ) if( ( x ) == 0 ) { fprintf(stderr, "ASSERT FAILED %s:%d\n", __FILE__, __LINE__); abort(); }

#endif /* FREERTOS_CONFIG_H */
