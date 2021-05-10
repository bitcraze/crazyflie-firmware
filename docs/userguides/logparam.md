---
title: Logging and parameter frameworks
page_id: logparam
---

The aim of the logging and parameter framework is to easily be able to
log data from the Crazyflie and to set variables during runtime.

## Table of content (TOC)

The variables that are available for the logging/parameter framework is
decided on compile-time for the Crazyflie firmware. Using C macros
variables can be made available to the framework below are two examples,
one for parameters and one for logging.

This will make the variables used to control the [LED-ring
expansion](https://wiki.bitcraze.io/projects:crazyflie2:expansionboards:ledring) available as
parameters. Note that they have different types and that *neffect* is
read-only.

``` {.c}
PARAM_GROUP_START(ring)
PARAM_ADD(PARAM_UINT8, effect, &effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT8, solidRed, &solidRed)
PARAM_ADD(PARAM_UINT8, solidGreen, &solidGreen)
PARAM_ADD(PARAM_UINT8, solidBlue, &solidBlue)
PARAM_ADD(PARAM_UINT8, headlightEnable, &headlightEnable)
PARAM_ADD(PARAM_FLOAT, glowstep, &glowstep)
PARAM_ADD(PARAM_FLOAT, emptyCharge, &emptyCharge)
PARAM_ADD(PARAM_FLOAT, fullCharge, &fullCharge)
PARAM_GROUP_STOP(ring)
```

This will make the variables for roll/pitch/yaw/thrust available for the
logging framework. These are the variables used to fill in the data in
the [Python cfclient FlightTab](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/).

    LOG_GROUP_START(stabilizer)
    LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
    LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
    LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
    LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
    LOG_GROUP_STOP(stabilizer)

During the compilation a table of content (TOC) is created that holds
all the available variables together with the type and access
restrictions. There\'s one TOC for each framework, one for logging and
one for parameters. When the client connects it will download the TOC to
know which variables can be used. It\'s then easy to use the [Python
API](https://github.com/bitcraze/crazyflie-lib-python) ([or another
API](https://wiki.bitcraze.io/doc:crazyflie:api:community) for accessing them.

All the variables have a name and belong to a group. So in the examples
above there\'s two groups defined: *ring* and *stabilizer*. To refer to
a variable use the naming convention *group.name*. If you would like to
log the *roll* variable in the *stabilizer* group it\'s access by
*stabilizer.roll*. And if you would like to set the *effect* variable in
the ring group it\'s accessed using *ring.effect*.

## Parameters

Using the parameter framework it\'s possible to both read and write
variables in run-time, but note the following:

-   There\'s no thread protection on reading/writing. Since the
    architecture is 32bit and the largest parameter you can have is
    32bit it\'s safe to write one variable. But if you write a group of
    variables that should be used together (like PID parameters) you
    might end up in trouble.
-   Only use the parameter framework to read variables that are set
    during start-up. If variables change during runtime then use the
    logging framework.
-   The reading or writing of a parameter can be done at any time once
    you are connected to the Crazyflie.

## Logging

The logging framework is used to log variables from the Crazyflie at a
specific interval. Instead of triggering a reading of the variables at
certain intervals, the framework is used to set up a logging
configuration to that will push data from the Crazyflie to the host.
After the host has connected to a Crazyflie and downloaded the TOC it
will be possible to setup one of these configurations. Once the
configuration is set up and started the Crazyflie will start pushing
data to the host. A configuration can be stopped and re-started again.
Since there\'s a finite amount of memory a configuration can be deleted
to make room for new ones.

Note the following for the logging framework:

-   Once a Crazyflie is connected you can set up new logging
    configurations. It\'s only possible to create/start/stop/remove
    configurations that\'s already created.
-   The interval for a logging configuration is specified in 10th of
    milliseconds.

Note that variables can be logged as different types from what they have
been declared as in the firmware. I.e if a variable is declared as a
uint32\_t you can log it as a uint8\_t (this is being done for the
motors in the UI).

### Logging using a function

It is possible to use a function to acquire log values instead of
reading from a memory location. The idea is to support more complex use cases
where, for instance computations are required to produce the log value.

The macro

```LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS)```

is used to add a log to a group. ```TYPE``` and ```NAME``` works the same way as for variable logging, but
```ADDRESS``` should point at a logByFunction_t struct instead of a variable. The struct contains
a function pointer to the function to call when acquiring data, as well as
a void pointer with user data passed that is passed on in the function call.

Example:

        float myLogValueFunction(uint32_t timestamp, void* data) {
            // Return the log value
            return 47.11;
        }

        logByFunction_t myLogger = {.aquireFloat = myLogValueFunction, .data = 0};

        LOG_GROUP_START(myGroup)
        LOG_ADD_BY_FUNCTION(LOG_FLOAT, myLog, &myLogger)
        LOG_GROUP_STOP(myGroup)

Note: The logging function is only called if the log is part of an active log configuration. It
will be called (approximately) at the interval that is setup in the log configuration.

### Logging rates

A common usecase is to log rates (events / s) and there are a set of macros to simplify this task.

The rate logger uses a struct to store data, it is called ```statsCntRateCounter_t``` and
is initialize using the ```STATS_CNT_RATE_INIT```macro. The struct contains a counter, time
information and the latest calculated rate. The initialization macro takes a parameter
that specifies the update interval (in ms) and defines how often the rate (delta count / delta time)
is calculated. The update interval should match the expected event rate to give meaningful results.

With the ```STATS_CNT_RATE_DEFINE``` macro, the struct can be defined and initialized in one line.

To register an event, that is to increase the counter, use the ```STATS_CNT_RATE_EVENT```macro.

Register the rate counter in a log group witht the ```STATS_CNT_RATE_LOG_ADD``` macro.

Example:

        static const uint32_t one_second = 1000;
        static statsCntRateLogger_t myCounter;
        static STATS_CNT_RATE_DEFINE(myOtherCounter, one_second);

        void myInit() {
            STATS_CNT_RATE_INIT(&myCounter, one_second);
        }

        void theFcnThatIWantTheCallRateFor() {
            ...
            STATS_CNT_RATE_EVENT(&myCounter);
            ...
            STATS_CNT_RATE_EVENT(&myOtherCounter);
        }

        LOG_GROUP_START(myGroup)
            // The logging type is implicitly LOG_FLOAT
            STATS_CNT_RATE_LOG_ADD(rtCall, &myCounter)
            STATS_CNT_RATE_LOG_ADD(rtCall2, &myOtherCounter)
        LOG_GROUP_STOP(myGroup)

Note: The rate computation function is called from the logging framework with the interval
specifed in the logging configuration. The rate, on the other hand, is calculated if the time since
last computation exceeds the configured update time of the rate logger, and if the logging intervall
is longer than the update intervall, updates will be done for each logging call.
