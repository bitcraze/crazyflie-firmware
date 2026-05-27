#define DEBUG_MODULE "ONBOARD_GUIDANCE"
#include "debug.h"

#include "autoconf.h"
#include "onboard_guidance.h"

#ifdef CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE
#include "crtp_commander_high_level.h"
#endif

#ifdef CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE
#define DEFAULT_GUIDANCE OnboardGuidanceTypeHighLevelCommander
#endif

static OnboardGuidanceType currentType = OnboardGuidanceTypeAutoSelect;

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  bool (*getSetpoint)(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep);
  void (*stop)(void);
  void (*tellState)(const state_t *state);
  void (*block)(bool doBlock);
  bool (*isDone)(void);
  const char *name;
} OnboardGuidanceFcns;

#ifdef CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE
static bool hlcTestAlwaysTrue(void) { return true; }
static void hlcStopVoid(void) { crtpCommanderHighLevelStop(); }
static void hlcBlockVoid(bool doBlock) { crtpCommanderBlock(doBlock); }
#endif


static OnboardGuidanceFcns guidanceFunctions[OnboardGuidanceType_COUNT] = {
  [OnboardGuidanceTypeAutoSelect] = {.name = "None"},
#ifdef CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE
  [OnboardGuidanceTypeHighLevelCommander] = {
    .init = crtpCommanderHighLevelInit,
    .test = hlcTestAlwaysTrue,
    .getSetpoint = crtpCommanderHighLevelGetSetpoint,
    .stop = hlcStopVoid,
    .tellState = crtpCommanderHighLevelTellState,
    .block = hlcBlockVoid,
    .isDone = crtpCommanderHighLevelIsTrajectoryFinished,
    .name = "HighLevelCommander",
  },
#endif
#ifdef CONFIG_ONBOARD_GUIDANCE_OOT
  [OnboardGuidanceTypeOot] = {
    .init = onboardGuidanceOutOfTreeInit,
    .test = onboardGuidanceOutOfTreeTest,
    .getSetpoint = onboardGuidanceOutOfTreeGetSetpoint,
    .stop = onboardGuidanceOutOfTreeStop,
    .tellState = onboardGuidanceOutOfTreeTellState,
    .block = onboardGuidanceOutOfTreeBlock,
    .isDone = onboardGuidanceOutOfTreeIsDone,
    .name = "OutOfTree",
  },
#endif
};

void onboardGuidanceInit(OnboardGuidanceType type) {
  if (type < 0 || type >= OnboardGuidanceType_COUNT) {
    return;
  }

  OnboardGuidanceType selected = type;

  if (selected == OnboardGuidanceTypeAutoSelect) {
    #if defined(CONFIG_ONBOARD_GUIDANCE_HIGHLEVEL_COMMANDER)
      selected = OnboardGuidanceTypeHighLevelCommander;
    #elif defined(CONFIG_ONBOARD_GUIDANCE_OOT)
      selected = OnboardGuidanceTypeOot;
    #elif defined(CONFIG_ONBOARD_GUIDANCE_HLC_ENABLE)
      selected = DEFAULT_GUIDANCE;
    #else
      #error "Auto-select requested but no onboard guidance is compiled in. Enable an onboard guidance backend via Kconfig."
    #endif
  }

  if (!guidanceFunctions[selected].init) {
    return;
  }

  currentType = selected;
  guidanceFunctions[currentType].init();

  DEBUG_PRINT("Using %s (%d) onboard guidance\n", onboardGuidanceGetName(), currentType);
}

bool onboardGuidanceTest(void) {
  return guidanceFunctions[currentType].test();
}

bool onboardGuidanceGetSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep) {
  return guidanceFunctions[currentType].getSetpoint(setpoint, state, stabilizerStep);
}

void onboardGuidanceStop(void) {
  guidanceFunctions[currentType].stop();
}

void onboardGuidanceTellState(const state_t *state) {
  guidanceFunctions[currentType].tellState(state);
}

void onboardGuidanceBlock(bool doBlock) {
  guidanceFunctions[currentType].block(doBlock);
}

bool onboardGuidanceIsDone(void) {
  return guidanceFunctions[currentType].isDone();
}

OnboardGuidanceType onboardGuidanceGetType(void) {
  return currentType;
}

const char* onboardGuidanceGetName(void) {
  return guidanceFunctions[currentType].name;
}
