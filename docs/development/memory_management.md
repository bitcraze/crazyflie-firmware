---
title: Memory management
page_id: memory_management
---

The RAM is a limiting resource and has to be managed to be used as efficiently as possible.

## Static VS dynamic allocation

Static memory allocation is done at compile time and the build system checks that the amount of memory that is required
actually is available in the Crazyflie. Dynamic memory, on the other hand, is requested and allocated in runtime using
the `malloc()` function. Dynamic memory allocation may fail (if there is not enough memory) and the error must be
handled in some way (currently by failing an assert which will reboot the Crazyflie).

In the Crazyflie firmware, we prefer static allocation over dynamic, mainly because it is checked at compile time and no
more error handling is required. Some calls to FreeRTOS do allocate a limited amount of dynamic RAM, but this is usually done
in the initialization phase and out of memory conditions should be detected quickly when booting the system.

## Static allocation

Static allocation of RAM happens when a variable is declared on a global scope (variables
declared in a function will end up on the stack). The actual allocation is done by the linker by
reserving space in the memory map in the appropriate location.

```
int myStaticallyAllocatedVariable;
static int myOtherStaticallyAllocatedVariable;

void aFunction() {
    int myVariableOnTheStack; // This ends up on the stack
    ...
}
```

Variables that are declared without an assignment will be set to zero at start up,
while variables with an assignment will be initialized to that value. The
values to use for initialized variables are stored in flash together with the
code and are copied to RAM at start up.

```
int zeroInitialized;
int initializedWithAValue = 17;
```

### RAM types

The MCU in the Crazyflie has two types of RAM, "normal" RAM and CCM (Core Coupled Memory).
The CCM and normal RAM are attached to different buses internally which gives them
different properties. The most significant difference is that the CCM can not be
used for DMA transfers, since the DMA unit does not have access to the bus used by CCM. The
normal RAM can be used for all types of tasks and should be used for normal development,
further more this is also the memory that will be used when writing "normal" code.

There is a fair amount of CCM in the system, and we want to use it to free up normal
RAM. The over all strategy is to use CCM for system related memory chunks that we
know will not be used for DMA, so that most users don't have to care about the
properties of CCM.

### Using the CCM

*Warning:* Some drivers use DMA to communicate with sensors, it will not work
to pass a pointer to CCM memory to such a driver.

There is a set of macros in [static_mem.h](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/static_mem.h)
that should be used to tell the compiler/linker to use the CCM. Note that only
zero initialization is supported for CCM.

```
int variableInNormalRam;  // Initialized to 0
int alsoVariableInNormalRam = 17;  // Initialized to 17
NO_DMA_CCM_SAFE_ZERO_INIT int variableInCcm; // Initialized to 0
NO_DMA_CCM_SAFE_ZERO_INIT int dontDoThisInCcm = 17; // WARNING! Will not work, will be initialized to 0! Will be silently accepted without warning.
````

Queues allocated with the `STATIC_MEM_QUEUE_ALLOC` macro will be allocated in the
CCM, all access to queue memory is done by copy and is safe.

Tasks allocated using the `STATIC_MEM_TASK_ALLOC` macro will allocate the
system buffers in CCM but the task stack in normal RAM. If no pointers
to stack memory will be used for DMA transfers, it is possible to
also allocate the stack in CCM by using the `STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE()`
macro instead.

Code that uses DMA through a public API should verify that pointers
passed in through the API do not point to CCM. Use `ASSERT_DMA_SAFE` for this
purpose to fail fast and indicate what the reason for the failed is.
