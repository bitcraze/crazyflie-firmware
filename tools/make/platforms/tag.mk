# Make configuration for the Tag platform

PLATFORM_HELP_tag = Tag platform, includes Roadrunner
PLATFORM_NAME_tag = Tag platform

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger, INDI
POWER_DISTRIBUTION ?= stock

######### COMPILE FLAGS ##########
CFLAGS += -DDECK_FORCE=bcDWM1000
CFLAGS += -DSENSORS_IGNORE_BAROMETER_FAIL
