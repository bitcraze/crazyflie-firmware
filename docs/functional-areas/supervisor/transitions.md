---
title: Supervisor transitions
page_id: supervisor_transitions
sort_order: 3
---

A state transition takes the state machine from one state to another. The [conditions bit field](conditions.md) is used to identify
which, if any transition to trigger. Transitions are defined as a list per state and are evaluated in order. If a
transition is triggered, the evaluation is terminated and no further transitions will be checked. That means
that the order of a transition list sets the priority, two transitions might have settings that makes both valid, but
the first one will "win".

A state transition is defined with a bit field of triggers. The bits in the trigger bit field are compared to the bits
in the current [conditions bit field](conditions.md) to see if they are set. There is also a bit field with negated triggers where the
bits in the condition bit field should be zero to trigger. To tie it together there is a trigger combiner that describes
the logical operation to use to decide whether the state transition should be triggered or not
* `supervisorAll` = all `trigger` bits must be set and all `negated trigger` bits must be zero in the condition bit field.
* `supervisorAny` = at least one `trigger` bit must be set or at least one `negated trigger` bits must be zero
* `supervisorAlways` = ignore the condition bit field and treat the condition as true
* `supervisorNever` = ignore the condition bit field and treat the condition as false

The second part of a state transition definition is a blocking functionality that works the same way as a trigger, but
is negated. It contains a bit field called blockers, a second bit field called negated blocker and a blocker combiner.
If the blocker evaluates to true, the trigger will not happen.

The final data of a state transition definition is the new state that the state machine will enter, if the trigger
condition is met and the blocking condition is not met.

The system of state transition definitions with triggers, negated triggers, blockers and negated blockers, match with
combiners provides a high degree of freedom. Note that it is possible to express a set of state transitions with trigger
conditions in multiple ways.
