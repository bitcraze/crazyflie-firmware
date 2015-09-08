/*******************************************************************************
Copyright © 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/



/* vl6180x_platform.h STM32 Nucelo F401 single device  using generic cci-i2c
 * trace via swo port some GnuArm eclipse toolset */

#ifndef VL6180x_PLATFORM
#define VL6180x_PLATFORM


#include "vl6180x_appcfg.h"
#include "vl6180x_def.h"



#define VL6180x_DEV_DATA_ATTR

#define ROMABLE_DATA
/*  #define ROMABLE_DATA  __attribute__ ((section ("user_rom"))) */



#if VL6180X_LOG_ENABLE
/*  dot not include non ansi here trace was a case :( */
#ifdef TRACE
#include "diag/trace.h"
extern volatile uint32_t g_TickCnt;
#define LOG_GET_TIME()  g_TickCnt
#else
/* these is nto stm32 vl6180x GNuArm eclpse build*/
#define trace_printf(...) (void)0
#define LOG_GET_TIME() (int)0 /* add your code here expect to be an integer native (%d) type  value  */
#endif



#define LOG_FUNCTION_START(fmt, ... ) \
    trace_printf("beg %s start @%d\t" fmt "\n", __func__, LOG_GET_TIME(), ##__VA_ARGS__)

#define LOG_FUNCTION_END(status)\
        trace_printf("end %s @%d %d\n", __func__, LOG_GET_TIME(), (int)status)

#define LOG_FUNCTION_END_FMT(status, fmt, ... )\
        trace_printf("End %s @%d %d\t"fmt"\n" , __func__, LOG_GET_TIME(), (int)status, ##__VA_ARGS__)

#define VL6180x_ErrLog(msg, ... )\
    do{\
        trace_printf("ERR in %s line %d\n" msg, __func__, __LINE__, ##__VA_ARGS__);\
    }while(0)

#else /* VL6180X_LOG_ENABLE no logging */
    void OnErrLog(void);
	#define LOG_FUNCTION_START(...) (void)0
	#define LOG_FUNCTION_END(...) (void)0
	#define LOG_FUNCTION_END_FMT(...) (void)0
    #define VL6180x_ErrLog(... ) OnErrLog()
#endif


#if  VL6180x_SINGLE_DEVICE_DRIVER
    typedef uint8_t VL6180xDev_t;

#else /* VL6180x_SINGLE_DEVICE_DRIVER */

    struct MyVL6180Dev_t {
        struct VL6180xDevData_t Data;
    #if I2C_BUFFER_CONFIG == 2
        uint8_t i2c_buffer[VL6180x_MAX_I2C_XFER_SIZE];
        #define VL6180x_GetI2cBuffer(dev, n) ((dev)->i2c_buffer)
    #endif
    };
    typedef struct MyVL6180Dev_t *VL6180xDev_t;

#define VL6180xDevDataGet(dev, field) (dev->Data.field)
#define VL6180xDevDataSet(dev, field, data) (dev->Data.field)=(data)

#endif /* #else VL6180x_SINGLE_DEVICE_DRIVER */

void VL6180x_PollDelay(VL6180xDev_t dev);

void DISP_ExecLoopBody(void);
#define VL6180x_PollDelay(dev)   DISP_ExecLoopBody();


#endif  /* VL6180x_PLATFORM */



