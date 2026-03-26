#pragma once

#include "stabilizer_types.h"

typedef enum {
  OnboardGuidanceTypeAutoSelect = 0,
#ifdef CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE
  OnboardGuidanceTypeHighLevelCommander,
#endif
#ifdef CONFIG_ONBOARD_GUIDANCE_OOT
  OnboardGuidanceTypeOot,
#endif
  OnboardGuidanceType_COUNT,
} OnboardGuidanceType;

void onboardGuidanceInit(OnboardGuidanceType type);
bool onboardGuidanceTest(void);
bool onboardGuidanceGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep);
void onboardGuidanceStop(void);
void onboardGuidanceTellState(const state_t *state);
void onboardGuidanceBlock(bool doBlock);
bool onboardGuidanceIsDone(void);
OnboardGuidanceType onboardGuidanceGetType(void);
const char* onboardGuidanceGetName(void);

#ifdef CONFIG_ONBOARD_GUIDANCE_OOT
void onboardGuidanceOutOfTreeInit(void);
bool onboardGuidanceOutOfTreeTest(void);
bool onboardGuidanceOutOfTreeGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep);
void onboardGuidanceOutOfTreeStop(void);
void onboardGuidanceOutOfTreeTellState(const state_t *state);
void onboardGuidanceOutOfTreeBlock(bool doBlock);
bool onboardGuidanceOutOfTreeIsDone(void);
#endif
