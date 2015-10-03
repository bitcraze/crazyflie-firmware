# st_obj.mk - Selection of the ST library objects to compile
# This file is part of the Crazy Flie control program
# Copyright (c) 2009, EAT-IT

VPATH+=$(STLIB)/STM32F4xx_StdPeriph_Driver/src/
ST_OBJ=
#ST_OBJ+=misc.o
ST_OBJ+=stm32f4xx_adc.o
#ST_OBJ+=stm32f4xx_bkp.o
#ST_OBJ+=stm32f4xx_can.o
#ST_OBJ+=stm32f4xx_crc.o
#ST_OBJ+=stm32f4xx_dac.o
ST_OBJ+=stm32f4xx_dbgmcu.o
ST_OBJ+=stm32f4xx_dma.o
ST_OBJ+=stm32f4xx_exti.o
ST_OBJ+=stm32f4xx_flash.o
#ST_OBJ+=stm32f4xx_fsmc.o
ST_OBJ+=stm32f4xx_gpio.o
ST_OBJ+=stm32f4xx_i2c.o
ST_OBJ+=stm32f4xx_iwdg.o
#ST_OBJ+=stm32f4xx_pwr.o
ST_OBJ+=stm32f4xx_rcc.o
#ST_OBJ+=stm32f4xx_rtc.o
#ST_OBJ+=stm32f4xx_sdio.o
ST_OBJ+=stm32f4xx_spi.o
ST_OBJ+=stm32f4xx_tim.o
ST_OBJ+=stm32f4xx_usart.o
ST_OBJ+=stm32f4xx_misc.o
#ST_OBJ+=stm32f4xx_wwdg.o
ST_OBJ+=stm32f4xx_syscfg.o
