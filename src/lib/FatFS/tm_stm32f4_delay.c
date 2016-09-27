/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32f4_delay.h"

__IO uint32_t TM_TimingDelay = 0;
__IO uint32_t TM_Time = 0;
__IO uint32_t TM_Time2 = 0;
volatile uint32_t mult;
uint8_t TM_DELAY_Initialized = 0;

/* Private structure */
typedef struct {
	uint8_t Count;
	TM_DELAY_Timer_t* Timers[DELAY_MAX_CUSTOM_TIMERS];
} TM_DELAY_Timers_t;

/* Custom timers structure */
static TM_DELAY_Timers_t CustomTimers;

void TM_DELAY_1msHandler(void) {
	uint8_t i;
	
	TM_Time++;
	if (TM_Time2 != 0x00) {
		TM_Time2--;
	}

	/* Check custom timers */
	for (i = 0; i < CustomTimers.Count; i++) {
		/* Check if timer is enabled */
		if (
			CustomTimers.Timers[i] &&          /*!< Pointer exists */
			CustomTimers.Timers[i]->Enabled && /*!< Timer is enabled */
			CustomTimers.Timers[i]->CNT > 0    /*!< Counter is not NULL */
		) {
			/* Decrease counter */
			CustomTimers.Timers[i]->CNT--;
			
			/* Check if count is zero */
			if (CustomTimers.Timers[i]->CNT == 0) {
				/* Call user callback function */
				CustomTimers.Timers[i]->Callback(CustomTimers.Timers[i]->UserParameters);
				
				/* Set new counter value */
				CustomTimers.Timers[i]->CNT = CustomTimers.Timers[i]->ARR;
				
				/* Disable timer if auto reload feature is not used */
				if (!CustomTimers.Timers[i]->AutoReload) {
					/* Disable counter */
					CustomTimers.Timers[i]->Enabled = 0;
				}
			}
		}
	}
}

void TM_DELAY_Init(void) {	
	/* Set initialized flag */
	TM_DELAY_Initialized = 1;
}

void TM_DELAY_EnableDelayTimer(void) {
	/* Check if library is even initialized */
	if (!TM_DELAY_Initialized) {
		return;
	}
}

void TM_DELAY_DisableDelayTimer(void) {
}

TM_DELAY_Timer_t* TM_DELAY_TimerCreate(uint32_t ReloadValue, uint8_t AutoReload, uint8_t StartTimer, void (*TM_DELAY_CustomTimerCallback)(void *), void* UserParameters) {
	TM_DELAY_Timer_t* tmp;
	
	/* Check if available */
	if (CustomTimers.Count >= DELAY_MAX_CUSTOM_TIMERS) {
		return NULL;
	}
	
	/* Try to allocate memory for timer structure */
	tmp = (TM_DELAY_Timer_t *) LIB_ALLOC_FUNC(sizeof(TM_DELAY_Timer_t));
	
	/* Check if allocated */
	if (tmp == NULL) {
		return NULL;
	}
	
	/* Fill settings */
	tmp->ARR = ReloadValue;
	tmp->CNT = tmp->ARR;
	tmp->AutoReload = AutoReload;
	tmp->Enabled = StartTimer;
	tmp->Callback = TM_DELAY_CustomTimerCallback;
	tmp->UserParameters = UserParameters;
	
	/* Increase number of timers in memory */
	CustomTimers.Timers[CustomTimers.Count++] = tmp;
	
	/* Return pointer to user */
	return tmp;
} 

void TM_DELAY_TimerDelete(TM_DELAY_Timer_t* Timer) {
	uint8_t i;
	uint32_t irq;
	TM_DELAY_Timer_t* tmp;
	
	/* Get location in array of pointers */
	for (i = 0; i < CustomTimers.Count; i++) {
		if (Timer == CustomTimers.Timers[i]) {
			break;
		}
	}
	
	/* Check for valid input */
	if (i == CustomTimers.Count) {
		return;
	}
	
	/* Save pointer to timer */
	tmp = CustomTimers.Timers[i];
	
	/* Get interrupt status */
	irq = __get_PRIMASK();

	/* Disable interrupts */
	__disable_irq();
	
	/* Shift array up */
	for (; i < (CustomTimers.Count - 1); i++) {
		/* Shift data to the left */
		CustomTimers.Timers[i] = CustomTimers.Timers[i + 1];
	}
	
	/* Decrease count */
	CustomTimers.Count--;
	
	/* Enable IRQ if necessary */
	if (!irq) {
		__enable_irq();
	}
	
	/* Free timer */
	LIB_FREE_FUNC(tmp);
}

TM_DELAY_Timer_t* TM_DELAY_TimerStop(TM_DELAY_Timer_t* Timer) {
	/* Disable timer */
	Timer->Enabled = 0;
	
	/* Return pointer */
	return Timer;
}

TM_DELAY_Timer_t* TM_DELAY_TimerStart(TM_DELAY_Timer_t* Timer) {
	/* Enable timer */
	Timer->Enabled = 1;
	
	/* Return pointer */
	return Timer;
}

TM_DELAY_Timer_t* TM_DELAY_TimerReset(TM_DELAY_Timer_t* Timer) {
	/* Reset timer */
	Timer->CNT = Timer->ARR;
	
	/* Return pointer */
	return Timer;
}

TM_DELAY_Timer_t* TM_DELAY_TimerAutoReload(TM_DELAY_Timer_t* Timer, uint8_t AutoReload) {
	/* Reset timer */
	Timer->AutoReload = AutoReload;
	
	/* Return pointer */
	return Timer;
}

TM_DELAY_Timer_t* TM_DELAY_TimerAutoReloadValue(TM_DELAY_Timer_t* Timer, uint32_t AutoReloadValue) {
	/* Reset timer */
	Timer->ARR = AutoReloadValue;
	
	/* Return pointer */
	return Timer;
}
