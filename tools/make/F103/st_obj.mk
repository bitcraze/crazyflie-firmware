# st_obj.mk - Selection of the ST library objects to compile
# This file is part of the Crazy Flie control program
# Copyright (c) 2009, EAT-IT

VPATH+=$(LIB)/STM32F10x_StdPeriph_Driver/src/
ST_OBJ=
ST_OBJ+=misc.o
ST_OBJ+=stm32f10x_adc.o
#ST_OBJ+=stm32f10x_bkp.o
#ST_OBJ+=stm32f10x_can.o
#ST_OBJ+=stm32f10x_crc.o
#ST_OBJ+=stm32f10x_dac.o
ST_OBJ+=stm32f10x_dbgmcu.o
ST_OBJ+=stm32f10x_dma.o
ST_OBJ+=stm32f10x_exti.o
ST_OBJ+=stm32f10x_flash.o
#ST_OBJ+=stm32f10x_fsmc.o
ST_OBJ+=stm32f10x_gpio.o
ST_OBJ+=stm32f10x_i2c.o
ST_OBJ+=stm32f10x_iwdg.o
#ST_OBJ+=stm32f10x_pwr.o
ST_OBJ+=stm32f10x_rcc.o
#ST_OBJ+=stm32f10x_rtc.o
#ST_OBJ+=stm32f10x_sdio.o
ST_OBJ+=stm32f10x_spi.o
ST_OBJ+=stm32f10x_tim.o
ST_OBJ+=stm32f10x_usart.o
#ST_OBJ+=stm32f10x_wwdg.o
