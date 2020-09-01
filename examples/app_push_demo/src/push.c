#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"

#define DEBUG_MODULE "PUSH"

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

static State state = idle;

static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

static const float velMax = 1.0f;
static const uint16_t radius = 300;

static const float height_sp = 0.2f;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

void appMain()
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));

  uint16_t idUp = logGetVarId("range", "up");
  uint16_t idLeft = logGetVarId("range", "left");
  uint16_t idRight = logGetVarId("range", "right");
  uint16_t idFront = logGetVarId("range", "front");
  uint16_t idBack = logGetVarId("range", "back");

  float factor = velMax/radius;

  //DEBUG_PRINT("%i", idUp);

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    //DEBUG_PRINT(".");
    uint16_t up = logGetUint(idUp);
    if (state == unlocked) {
      uint16_t left = logGetUint(idLeft);
      uint16_t right = logGetUint(idRight);
      uint16_t front = logGetUint(idFront);
      uint16_t back = logGetUint(idBack);

      uint16_t left_o = radius - MIN(left, radius);
      uint16_t right_o = radius - MIN(right, radius);
      float l_comp = (-1) * left_o * factor;
      float r_comp = right_o * factor;
      float velSide = r_comp + l_comp;

      uint16_t front_o = radius - MIN(front, radius);
      uint16_t back_o = radius - MIN(back, radius);
      float f_comp = (-1) * front_o * factor;
      float b_comp = back_o * factor;
      float velFront = b_comp + f_comp;

      uint16_t up_o = radius - MIN(up, radius);
      float height = height_sp - up_o/1000.0f;

      /*DEBUG_PRINT("l=%i, r=%i, lo=%f, ro=%f, vel=%f\n", left_o, right_o, l_comp, r_comp, velSide);
      DEBUG_PRINT("f=%i, b=%i, fo=%f, bo=%f, vel=%f\n", front_o, back_o, f_comp, b_comp, velFront);
      DEBUG_PRINT("u=%i, d=%i, height=%f\n", up_o, height);*/

      if (1) {
        setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        commanderSetSetpoint(&setpoint, 3);
      }

      if (height < 0.1f) {
        state = stopping;
        DEBUG_PRINT("X\n");
      }

    } else {

      if (state == stopping && up > stoppedTh) {
        DEBUG_PRINT("%i", up);
        state = idle;
        DEBUG_PRINT("S\n");
      }

      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }

      if (up > unlockThHigh && state == lowUnlock) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }

      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}
