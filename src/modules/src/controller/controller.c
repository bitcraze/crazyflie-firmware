#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_mellinger.h"
#include "controller_indi.h"
#include "controller_brescianini.h"
#include "controller_lee.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypePID
static ControllerType currentController = ControllerTypeAutoSelect;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  void (*enterPreciseLand)(void);
  void (*exitPreciseLand)(void);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
  {.init = 0, .test = 0, .update = 0, .enterPreciseLand = 0, .exitPreciseLand = 0, .name = "None"}, // Any
  {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid,
   .enterPreciseLand = controllerPidEnterPreciseLand, .exitPreciseLand = controllerPidExitPreciseLand, .name = "PID"},
  {.init = controllerMellingerFirmwareInit, .test = controllerMellingerFirmwareTest, .update = controllerMellingerFirmware,
   .enterPreciseLand = controllerMellingerEnterPreciseLand, .exitPreciseLand = controllerMellingerExitPreciseLand, .name = "Mellinger"},
  {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI,
   .enterPreciseLand = controllerINDIEnterPreciseLand, .exitPreciseLand = controllerINDIExitPreciseLand, .name = "INDI"},
  {.init = controllerBrescianiniInit, .test = controllerBrescianiniTest, .update = controllerBrescianini,
   .enterPreciseLand = controllerBrescianiniEnterPreciseLand, .exitPreciseLand = controllerBrescianiniExitPreciseLand, .name = "Brescianini"},
  {.init = controllerLeeFirmwareInit, .test = controllerLeeFirmwareTest, .update = controllerLeeFirmware,
   .enterPreciseLand = controllerLeeEnterPreciseLand, .exitPreciseLand = controllerLeeExitPreciseLand, .name = "Lee"},
  #ifdef CONFIG_CONTROLLER_OOT
  {.init = controllerOutOfTreeInit, .test = controllerOutOfTreeTest, .update = controllerOutOfTree, .name = "OutOfTree"},
  #endif
};


void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  ControllerType selectedController = controller;

  if (selectedController == ControllerTypeAutoSelect) {
    #if defined(CONFIG_CONTROLLER_PID)
      selectedController = ControllerTypePID;
    #elif defined(CONFIG_CONTROLLER_INDI)
      selectedController = ControllerTypeINDI;
    #elif defined(CONFIG_CONTROLLER_MELLINGER)
      selectedController = ControllerTypeMellinger;
    #elif defined(CONFIG_CONTROLLER_BRESCIANINI)
      selectedController = ControllerTypeBrescianini;
    #elif defined(CONFIG_CONTROLLER_LEE)
      selectedController = ControllerTypeLee;
    #elif defined(CONFIG_CONTROLLER_OOT)
      selectedController = ControllerTypeOot;
    #else
      selectedController = DEFAULT_CONTROLLER;
    #endif
  }

  currentController = selectedController;

  initController();

  DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}

ControllerType controllerGetType(void) {
  return currentController;
}

static void initController() {
  controllerFunctions[currentController].init();
}

bool controllerTest(void) {
  return controllerFunctions[currentController].test();
}

void controller(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, stabilizerStep);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}

void controllerEnterPreciseLand(void) {
  if (controllerFunctions[currentController].enterPreciseLand) {
    controllerFunctions[currentController].enterPreciseLand();
  }
}

void controllerExitPreciseLand(void) {
  if (controllerFunctions[currentController].exitPreciseLand) {
    controllerFunctions[currentController].exitPreciseLand();
  }
}
