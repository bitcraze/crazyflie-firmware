#include "controller_force_torque_external.h"

void controllerForceTorqueExternalInit(void) {}

bool controllerForceTorqueExternalTest(void) { return true; }

void controllerForceTorqueExternal(control_t *control,
                                   const setpoint_t *setpoint,
                                   const sensorData_t *sensors,
                                   const state_t *state,
                                   const uint32_t tick)
{
  (void)sensors;
  (void)state;
  (void)tick;

  if (!setpoint->forceTorqueSI.valid) {
    return;
  }

  control->controlMode = controlModeForceTorque;

  // FIRST BRING-UP mapping (you will scale later)
  control->thrust  = setpoint->forceTorqueSI.thrust_N;
  control->torqueX = setpoint->forceTorqueSI.torque_Nm[0];
  control->torqueY = setpoint->forceTorqueSI.torque_Nm[1];
  control->torqueZ = setpoint->forceTorqueSI.torque_Nm[2];
}
