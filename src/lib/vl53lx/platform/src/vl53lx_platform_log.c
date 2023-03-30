
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <malloc.h>
#include <vl53lx_platform_log.h>
#include <vl53lx_platform_user_config.h>


#ifdef VL53LX_LOG_ENABLE

	char * _trace_filename = NULL;
	FILE *_tracefile = NULL;

	uint32_t _trace_level     = VL53LX_TRACE_LEVEL_WARNING;
	uint32_t _trace_modules   = VL53LX_TRACE_MODULE_NONE;
	uint32_t _trace_functions = VL53LX_TRACE_FUNCTION_ALL;

	int8_t VL53LX_trace_config(
		char *filename,
		uint32_t modules,
		uint32_t level,
		uint32_t functions)
	{
		int8_t status = 0;















		if (((filename != NULL) && (_tracefile == NULL)) && strcmp(filename,""))
		{
			_tracefile = fopen(filename, "w+");



			if ( _tracefile != NULL )
			{
				_trace_filename = (char*)malloc((strlen(filename) + 1) * sizeof(char));
				strcpy(_trace_filename, filename);
			}
			else
			{
				printf("VL53LX_trace_config(): failed to open log file (%s)\n", filename);
				status = 1;
			}
		}

		_trace_modules   = modules;
		_trace_level     = level;
		_trace_functions = functions;

		return status;
	}

	void VL53LX_trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
	{
		if ( ((level <=_trace_level) && ((module & _trace_modules) > 0))
			|| ((function & _trace_functions) > 0) )
		{
			va_list arg_list;
			char message[VL53LX_MAX_STRING_LENGTH];

			va_start(arg_list, format);
			vsnprintf(message, VL53LX_MAX_STRING_LENGTH-1, format, arg_list);
			va_end(arg_list);

			if (_tracefile != NULL)
			{
				fprintf(_tracefile, message);
			}
			else
			{
				printf(message);
			}





		}
	}


	uint32_t VL53LX_get_trace_functions(void)
	{
		return _trace_functions;
	}


	void VL53LX_set_trace_functions(uint32_t function)
	{
		_trace_functions = function;
	}


	uint32_t VL53LX_clock(void)
	{

		uint32_t tick_count_ms = (uint32_t)clock();
		return tick_count_ms;
	}
#endif

