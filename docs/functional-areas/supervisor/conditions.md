---
title: Supervisor conditions
page_id: supervisor_conditions
sort_order: 1
---

A condition express the state of some part of the system, for instance if we are flying, if the system is armed or
tumbled. A condition is a single bit and can thus only be true or false.

All conditions are collected in a bit field that expresses the full state of the system, or at least all the parts
that are relevant to the supervisor.

All conditions can be found in the `src/modules/interface/supervisor_state_machine.h` file.

The condition bit field is used to trigger [state transitions](transitions.md).
