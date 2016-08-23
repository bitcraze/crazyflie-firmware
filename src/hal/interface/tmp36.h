#ifndef __TMP36_H__
#define __TMP36_H__

#include <stdint.h>

/**
 * \def tmp36_ENABLED
 * Enable the tmp36 measurement subsystem.
 */
#define tmp36_ENABLED

/**
 * \def tmp36_TASK_FREQ
 * The frequency the temp task runs at.
 */
#define tmp36_TASK_FREQ 5

/**
 * Number of samples in the sliding window. Used for average calculation.
 */
#define tmp36_SWIN_SIZE 9

/**
 * \def tmp36_LOG_ENABLED
 * to enable log variables.
 */
#define tmp36_LOG_ENABLED

void tmp36Init(void);

#endif
