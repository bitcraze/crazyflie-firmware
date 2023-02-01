#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_mellinger.h"
#include "controller_indi.h"
#include "controller_brescianini.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypePID
static ControllerType currentController = ControllerTypeAutoSelect;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
  {.init = 0, .test = 0, .update = 0, .name = "None"}, // Any
  {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid, .name = "PID"},
  {.init = controllerMellingerFirmwareInit, .test = controllerMellingerFirmwareTest, .update = controllerMellingerFirmware, .name = "Mellinger"},
  {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI"},
  {.init = controllerBrescianiniInit, .test = controllerBrescianiniTest, .update = controllerBrescianini, .name = "Brescianini"},
  #ifdef CONFIG_CONTROLLER_OOT
  {.init = controllerOutOfTreeInit, .test = controllerOutOfTreeTest, .update = controllerOutOfTree, .name = "OutOfTree"},
  #endif
};


void controllerInit(ControllerType controller) {
  if (controller < 0 || controller >= ControllerType_COUNT) {
    return;
  }

  currentController = controller;

  if (ControllerTypeAutoSelect == currentController) {
    currentController = DEFAULT_CONTROLLER;
  }

  #if defined(CONFIG_CONTROLLER_PID)
    #define CONTROLLER ControllerTypePID
  #elif defined(CONFIG_CONTROLLER_INDI)
    #define CONTROLLER ControllerTypeINDI
  #elif defined(CONFIG_CONTROLLER_MELLINGER)
    #define CONTROLLER ControllerTypeMellinger
  #elif defined(CONFIG_CONTROLLER_BRESCIANINI)
    #define CONTROLLER ControllerTypeBrescianini
  #else
    #define CONTROLLER ControllerTypeAutoSelect
  #endif

  ControllerType forcedController = CONTROLLER;
  if (forcedController != ControllerTypeAutoSelect) {
    DEBUG_PRINT("Controller type forced\n");
    currentController = forcedController;
  }

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

void controller(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, tick);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
