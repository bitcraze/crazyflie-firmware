---
title: Supervisor states
page_id: supervisor_states
sort_order: 2
---

## State diagram

![supervisor state diagram](/docs/images/supervisor_state_diagram.svg){:align-center}

All states can be found in the `src/modules/interface/supervisor_state_machine.h` file.

## State transitions

Transitions between states are expressed as a struct containing the next state and two bit fields with requirements for
the transition to be valid. One bitfield contains the [conditions](conditions.md) that **must** be fulfilled and the
other the [conditions](conditions.md) that **must not** be fulfilled, all other [conditions](conditions.md) are ignored.

For each state there is a list of possible state transitions. The supervisor goes through the list for the current state
and checks if the current [conditions](conditions.md) match the bitfields of the transitions. The first state transition
that matches is used to change state of the supervisor. If no valid transition is found, the state is not changed.

State transitions are defined in `src/modules/src/supervisor_state_machine.c`
