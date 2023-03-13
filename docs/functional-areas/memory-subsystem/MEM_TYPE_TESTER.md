---
title: Test - MEM_TYPE_TESTER
page_id: mem_type_tester
---

This memory mapping is used for testing.

Reading from the memory returns the lower 8 bits of the address, for instance reading address 0x0017 will return
0x17, and reading from address 0x4711 will return 0x11

The same pattern is expected when writing to the memory and if the written data is not correct, a message will be
written to the console log.
