---
title: The memory subsystem
page_id: memory_subsystem
---

The memory subsystem is intended to be used for transfering larger chunks of data
between a Crazyflie and a client where real time characteristics are not important.

The memory subsystem uses the ```CRTP_PORT_MEM``` port for communication and supports basic read and write functionality of raw binary data. One packet on the CRTP level contains data for one read or write operation with information on address, size and type. A larger read or write is split up in multiple operations, where each sub-operation fits into one
CRTP packet. The sub-operations should be done in order (from lower address to higher) and
some read handlers may rely on this to update caches or similar when receiving the last part of a block.

The type can be seen as the highest level of abstraction and indicates which memory
the read/write operation will act on. There are a number of memory types (defined in ```MemoryType_t```) which each implements a read and/or write paradigm of selected parts of a virtual memory space, for a functional area of the system. A read/write
implementation may be mapped directly to a block of RAM in the Crazyflie, but it can
also be connected to other data structures through function calls where the
virtual memory map is simply a means of organaizing data for transfer.

A module in the Crazyflie firmware registeres handlers for read and/or write operations
and the framwork calls the handlers with appropriate values when memory operations are
initiated from a client.

## Memory types and mappings

{% sub_page_menu %}
