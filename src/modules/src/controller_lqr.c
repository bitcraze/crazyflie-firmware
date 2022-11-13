#include "controller_lqr.h"

#define m 0.027
#define g 9.81

#define M_PI   3.14159265358979323846

void controllerLqrInit(void){
    //attitudeControllerInit(ATTITUDE_UPDATE_DT);
    //positionControllerInit();
}

bool controllerLqrTest(void){
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

void controllerLqr(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  float const deg2millirad = (float)M_PI / 180.0f;
  float state_rateRoll = sensors->gyro.x * deg2millirad;
  float state_ratePitch = -sensors->gyro.y * deg2millirad;
  float state_rateYaw = sensors->gyro.z * deg2millirad;

  float K_lqr[4][12] = {
    {0.0, 0.0, 4.6445, 0.0, 0.0, 0.0, 0.0, 0.0, 1.367, 0.0, 0.0, 0.0},
    {0.0, -0.0031, 0.0, 0.0128, 0.0, 0.0, 0.0, -0.003, 0.0, 0.0018, 0.0, 0.0}, 
    {0.0032, 0.0, 0.0, 0.0, 0.0132, 0.0, 0.0031, 0.0, 0.0, 0.0, 0.0019, 0.0}, 
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0086, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0028}};
  float u[4];
  float dt = 0.01; 
  float e1 = setpoint->position.x *  - state->position.x;
  float e2 = setpoint->position.y - state->position.y;
  float e3 = setpoint->position.z - state->position.z;

  float e4 = setpoint->attitude.roll - state->attitude.roll;
  float e5 = setpoint->attitude.pitch - state->attitude.pitch;
  float e6 = setpoint->attitude.yaw - state->attitude.yaw;

  float e7 = setpoint->velocity.x - state->velocity.x;
  float e8 = setpoint->velocity.y - state->velocity.y;
  float e9 = setpoint->velocity.z - state->velocity.z;

  float e10 = setpoint->attitudeRate.roll - state_rateRoll;
  float e11 = setpoint->attitudeRate.pitch - state_ratePitch;
  float e12 = setpoint->attitudeRate.yaw - state_rateYaw;
  float error[12] = {e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12};
  int i, j, k = 0; 
  float res = 0;
  
  while (i < 4){
    while (j < 12 && k < 12){
      res += K_lqr[i][j] * error[k];
      j++;
      k++; 
    }

    u[i] = res;
    i++;

    j = 0; k = 0;
    res = 0;
  }

  /* feedback */
  control->thrust = u[0] + m * g;
  control->roll = u[1];
  control->pitch = u[2];
  control->yaw = u[3];
}
