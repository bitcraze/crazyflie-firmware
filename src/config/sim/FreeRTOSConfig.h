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
/* configUSE_TIMERS was off through Phase 4.4 (no FreeRTOS software timers
 * used yet -- 4.0's ledseq_sim.c no-op'd the whole LED-sequence surface
 * rather than turn these on for no client-visible benefit). Phase 4.5
 * (Logging) is the first genuine need: log.c's periodic log blocks are
 * built on xTimerCreateStatic()/xTimerChangePeriod()/xTimerStart() et al.,
 * with no sim-friendly alternative to swap in (unlike ledseq, there's no
 * "narrower stub" that still streams periodic values as Use case 3 needs).
 * static_mem.c's vApplicationGetTimerTaskMemory() already referenced
 * configTIMER_TASK_STACK_DEPTH unconditionally at compile time regardless
 * of this setting, so no new define was needed for that part. */
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
#define configUSE_APPLICATION_TASK_TAG         0
#define configSUPPORT_STATIC_ALLOCATION        1
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

/* Milliseconds to OS ticks, matching mainline src/config/FreeRTOSConfig.h --
 * configTICK_RATE_HZ above is 1000, so these are 1:1. */
#define M2T(X) ((unsigned int)(X))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X))
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)

#endif /* FREERTOS_CONFIG_H */
