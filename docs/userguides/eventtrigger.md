---
title: Event Trigger framework
page_id: eventtrigger
---

The aim of the event trigger framework is to be able to log events and their payloads
synchronously. Thus, event triggers are closely related to the [Logging Framework](/docs/userguides/logparam.md), which
logs data asynchronously. One key use-case of the event trigger framework is to analyze different state estimators.
For each measurement that is pushed into the current state estimator, an event with the appropriate payload containing
the measurement can be stored for later analysis.

## Add Event Triggers in the Firmware

In order to add a new event trigger named `myEvent` with a payload of two variables, you have to add the following near the top of your *.c file:

``` {.c}
#include "eventtrigger.h"

// declares eventTrigger_myEvent and eventTrigger_myEvent_payload
EVENTTRIGGER(myEvent, uint8, var1, float, var2)
```

The EVENTTRIGGER macro is variadic, i.e. it can take a varying number of arguments. It is also valid to have a event trigger without any payload.
Internally, the macro generates code that declares several static variables. Details can be found in [eventtrigger.h](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/eventtrigger.h).

Once the event trigger is defined, it can be triggered anywhere in the source code of the same *.c file:

``` {.c}
// set the payload
eventTrigger_myEvent_payload.var1 = some_value_related_to_the_event;
eventTrigger_myEvent_payload.var2 = another_value_related_to_the_event;
// trigger the event
eventTrigger(&eventTrigger_myEvent);
```

Users can configure to log additional regular logging variables for each event, so the payload should be limited to mandatory data that identifies the event.
For example, if a new measurement is enqueued in the state estimator, the actual measurement should be included as payload, while the (constant) standard deviation 
should not be part of it.

## Using Event Triggers

Currently, the only backend for event triggers is the **uSD-card deck**. You can find a description of how to configure
and analyze the events on the usage tab of [the uSD-card deck product page](https://www.bitcraze.io/products/micro-sd-card-deck/).
