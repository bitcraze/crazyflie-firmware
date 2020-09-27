---
title: Adding a new system task
page_id: systemtask
---

**First check out if you can use the [new app layer](/docs/userguides/app_layer.md), which might be enough for your purpose already**


This howto describes how to create a new system task.
A FreeRTOS task is similar to a thread on a standard operating system.
It has its own function call stack, and can be preempted by the RTOS scheduler.
Typical hazards and techniques of shared-memory multithreaded programming apply.

A new task is most often needed when adding a fundamentally new subsystem
to the Crazyflie firmware.

If the new subsystem needs to do
significant computation in response to inputs such as CRTP radio messages, it
should receive those inputs on a queue and perform the computation within
its own task instead of blocking the radio task.
In this example, we will set up the skeleton for such a new subsystem.



Development environment
-----------------------

You should have the
[crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) and
[crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python)
cloned in the same folder. If you are using the [Bitcraze
VM](https://github.com/bitcraze/bitcraze-vm) this is already the case.

The Crazyflie firmware should be on Master branch.

For the rest of the howto you will work in the crazyflie-firmware
project.


Setting the task constants
--------------------------

Each task has a few constants that are stored globally in `config/config.h`.

First, set the task priority. The FreeRTOS scheduler always prefers to run
a higher-priority task. High priorities should be reserved for very
time-sensitive tasks like the main stabilizer loop.

In `config.h`, after the existing task priorities, add the line

``` {.c}
#define EXAMPLE_TASK_PRI        1
```

Next, each task has a name.
In `config.h`, after the existing task names, add the line

``` {.c}
#define EXAMPLE_TASK_NAME       "EXAMPLE"
```

Finally, we must select a fixed size for the task's function call stack.
`configMINIMAL_STACK_SIZE` gives a fairly small stack.
Tasks that do lots of computation will almost certainly need larger stacks.

In `config.h`, after the existing task stack sizes, add the line

``` {.c}
#define EXAMPLE_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
```


Implementing the task
---------------------

High level tasks (those that do not directly talk to hardware) usually go in
the `modules` directory. We will walk through the sections needed in a
new file `modules/src/example.c`.


First, include the necessary system header files:

``` {.c}
#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
```

### File-static variables

Next, we statically allocate the queue to hold inputs from
other parts of the system such as the radio.
Here we just hold integers in the queue. Often it will be a struct instead.
A queue of length 1 is appropriate if new inputs supersede old ones.
For example, if the input controls the current LED color,
we do not care which other colors were requested in the past.

Note that the `STATIC_MEM_*_ALLOC` family of macros expand to static
variable declarations, and can be used outside of any function scope.

``` {.c}
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(int));
```


Next, we allocate the task call stack and other necessary memory for OS
bookkeeping.
The FreeRTOS task API requires that task functions take a `void *` argument.

``` {.c}
static void exampleTask(void*);
STATIC_MEM_TASK_ALLOC(exampleTask, EXAMPLE_TASK_STACKSIZE);
```

### Functions

Next, we implement the `Init()` function.
This should contain all of the FreeRTOS allocations and initializations
that need to last as long as the firmware is running.
Note the use of the constants we defined in `config.h`.
Later in this howto, we will need to edit another file to call the `Init()`.

``` {.c}
static bool isInit = false;

void exampleTaskInit() {
  inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);
  // TODO
  STATIC_MEM_TASK_CREATE(exampleTask, exampleTask, EXAMPLE_TASK_NAME, NULL, EXAMPLE_TASK_PRI);
  isInit = true;
}
```

Next, we implement the `Test()` function.
Usually this does not do much, besides verify that `Init()` has been called.
Later in this howto, we will need to edit another file to call the `Test()`.

``` {.c}
bool exampleTaskTest() {
  return isInit;
}
```


Now we come to the main task function.
It is usually an infinite loop that waits on inputs from something.
In our case, we wait on inputs from the queue.
Our `xQueueReceive` uses the special timeout value `portMAX_DELAY`, which means
that the `xQueueReceive` call will (potentially) block forever and only return
when the queue is not empty.
Other tasks will use integer or zero timeout values
if they have some work to do regardless of whether or not inputs are received.

``` {.c}
static void exampleTask(void* parameters) {
  DEBUG_PRINT("Example task main function is running!");
  while (true) {
    int input;
    if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
      // Respond to input here!
    }
  }
}
```


Finally, we implement the public function that is called by other parts of the
system to pass inputs to our new subsystem.

`xQueueOverwrite` specifies that, if the queue is full, we overwrite the
existing contents instead of blocking on the queue's consumer to remove
an item from the other end of the queue.
In subsystems where we do not want to throw away inputs from the queue,
use the `xQueueSend*` family instead.

Subsystems should not expose their queues directly.
It should always be wrapped in a function, like this.

``` {.c}
void exampleTaskEnqueueInput(int value) {
  xQueueOverwrite(inputQueue, &value);
}
```

See [the FreeRTOS docs](https://freertos.org/Embedded-RTOS-Queues.html)
for more details on the many `xQueue` API functions available.

> **Design Note:** For complex subsystems, consider implementing the
> algorithmic core as a library that does not depend on FreeRTOS- and
> ARM-related headers, and calling this library from your task.
> This makes it easier to write unit tests for your algorithmic core
> that can be compiled, run, and debugged on a PC.


Writing the public interface
----------------------------

Subsystems should expose a public interface to other parts of the firmware
that hide implementation details as much as is practical.
We create the new file in `modules/interface/example.h`:

``` {.c}
#ifndef __EXAMPLE_TASK_H__
#define __EXAMPLE_TASK_H__

#include <stdbool.h>

void exampleTaskInit();
bool exampleTaskTest();

void exampleTaskEnqueueInput(int value);

#endif // __EXAMPLE_TASK_H__
```

In a real task, make sure to comment the public API thoroughly.


Initializing the task
---------------------

In `modules/src/system.c`, make the following changes.

Include the header for our new task:

``` {.c}
#include "example.h"
```


In `systemTask()`, after the lines where the other `Init()` functions
are called, add the line

``` {.c}
exampleTaskInit();
```


In `systemTask()`, after the lines where the other `pass &= ...Test()` lines,
add the line
``` {.c}
pass &= exampleTaskTest();
```


Adding the task to the build
----------------------------

Add this to the Makefile, after the end of the `Modules` block:

``` {.make}
PROJ_OBJ += example.o
```


Compile, flash and run!
-----------------------

Now the last step is to compile and flash your new firmware. Launch the
following commands in a shell:

``` {.bash}
crazyflie-firmware$ make
crazyflie-firmware$ make cload
```

The output will be similar to the following:

``` {.bash}
crazyflie-firmware$ make
(...)
  CC    hello.o
(...)
Build for the CF2 platform!
Build 22:f8243162f727 (2020.04 +22) MODIFIED
Version extracted from git
Crazyloader build!
Flash |  218132/1032192 (21%),  814060 free | text: 213024, data: 5108, ccmdata: 0
RAM   |   71564/131072  (55%),   59508 free | bss: 66456, data: 5108
CCM   |   43528/65536   (66%),   22008 free | ccmbss: 43528, ccmdata: 0
crazyflie-firmware$ make cload
../crazyflie-clients-python/bin/cfloader flash cf2.bin stm32-fw
Restart the Crazyflie you want to bootload in the next
 10 seconds ...
 done!
Connected to bootloader on Crazyflie 2.0 (version=0x10)
Target info: nrf51 (0xFE)
Flash pages: 232 | Page size: 1024 | Buffer pages: 1 | Start page: 88
144 KBytes of flash available for firmware image.
Target info: stm32 (0xFF)
Flash pages: 1024 | Page size: 1024 | Buffer pages: 10 | Start page: 16
1008 KBytes of flash available for firmware image.

Flashing 1 of 1 to stm32 (fw): 161867 bytes (159 pages) ..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10..........10.........9
Reset in firmware mode ...
$
```

Now you can connect your Crazyflie with the client and see the
"Example task main function is running!" in the debug console.
