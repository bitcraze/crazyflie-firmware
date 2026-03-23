#pragma once

#include "stabilizer_types.h"

#ifdef CONFIG_ONBOARD_GUIDANCE_OOT
void onboardGuidanceOutOfTreeInit(void);
bool onboardGuidanceOutOfTreeTest(void);
bool onboardGuidanceOutOfTreeGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep);
void onboardGuidanceOutOfTreeStop(void);
void onboardGuidanceOutOfTreeTellState(const state_t *state);
#endif
