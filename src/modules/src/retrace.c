#include "FreeRTOS.h"
#include "timers.h"
#include <float.h>
#include <math.h>
#include "sitaw.h"
#include "retrace.h"
#include "sound.h"
#include "commander.h"

#include "debug.h"
#include "estimator_kalman.h"

#define DEBUG_MODULE "RETRACE"


typedef enum {
  ST_WAIT_POS_LOCK = 0,
  ST_RECORD_TRACE,
  ST_RETRACE,
  ST_STOP
} rt_state_t;


static xTimerHandle timer;
static bool isInit = false;

static setpoint_t setpoint;


static void retraceTimer(xTimerHandle timer);
static void freeFallDetected();
static void changeState(rt_state_t newState);
static void handleStateWaitPosLock();
static void handleStateRecordTrace();
static void handleStateRetrace();
static void handleStateStop();
static void moveSetPoint(float x, float y, float z);

typedef struct {
  float x;
  float y;
  float z;
  uint32_t count;
} t_position;

#define NR_OF_POSITIONS 2000
static t_position positions[NR_OF_POSITIONS];
static int nextPosIndex = 0;


static rt_state_t state = ST_WAIT_POS_LOCK;

static bool startReplay = false;

static int lockCount = 0;
static float lXMin, lYMin, lZMin, lXMax, lYMax, lZMax;
#define LOCK_THRESHOLD 0.001f

static void resetLockData() {
    lXMax = FLT_MIN;
    lYMax = FLT_MIN;
    lZMax = FLT_MIN;

    lXMin = FLT_MAX;
    lYMin = FLT_MAX;
    lZMin = FLT_MAX;
}


void retraceInit(void)
{
  if (isInit) {
    return;
  }

  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;
  setpoint.mode.yaw = modeAbs;
  setpoint.mode.roll = modeDisable;
  setpoint.mode.pitch = modeDisable;

  sitAwRegisterFFCallback(freeFallDetected, 0);
  resetLockData();

  timer = xTimerCreate("RetraceTimer", M2T(100), pdTRUE, NULL, retraceTimer);
  xTimerStart(timer, 100);

  isInit = true;
}

bool retraceTest(void)
{
  return isInit;
}

static void retraceTimer(xTimerHandle timer)
{
  switch(state) {
    case ST_WAIT_POS_LOCK:
      handleStateWaitPosLock();
      break;

    case ST_RECORD_TRACE:
      handleStateRecordTrace();
      break;

    case ST_RETRACE:
      handleStateRetrace();
      break;

    case ST_STOP:
      handleStateStop();
      break;

    default:
      DEBUG_PRINT("Unexpected state!\n");
      break;
  }
}

static void changeState(rt_state_t newState) {
  DEBUG_PRINT("Go to state %d\n", newState);
  state = newState;
}

static void freeFallDetected() {
  startReplay = true;

  moveSetPoint(positions[nextPosIndex - 1].x, positions[nextPosIndex - 1].y, positions[nextPosIndex - 1].z);
}


static bool hasLock() {
  bool result = false;

  if (lockCount < 50) {
    float x = getVarPX();
    float y = getVarPY();
    float z = getVarPZ();

    lXMax = fmaxf(lXMax, x);
    lYMax = fmaxf(lYMax, y);
    lZMax = fmaxf(lZMax, z);

    lXMin = fminf(lXMax, x);
    lYMin = fminf(lYMin, y);
    lZMin = fminf(lZMin, z);
  } else {
    result = ((lXMax - lXMin) < LOCK_THRESHOLD) && ((lYMax - lYMin) < LOCK_THRESHOLD) && ((lZMax - lZMin) < LOCK_THRESHOLD);

    lockCount = 0;
    resetLockData();
  }

  lockCount++;

  return result;
}

#define DELTA_THRESHOLD 0.10
static void recordPosition() {
  bool isNewPosition = true;

  float X = getX();
  float Y = getY();
  float Z = getZ();

  if (nextPosIndex > 0) {
    float prevX = positions[nextPosIndex - 1].x;
    float prevY = positions[nextPosIndex - 1].y;
    float prevZ = positions[nextPosIndex - 1].z;

    isNewPosition = (fabs(prevX - X) > DELTA_THRESHOLD ||
    fabs(prevY - Y) > DELTA_THRESHOLD ||
    fabs(prevZ - Z) > DELTA_THRESHOLD);
  }

  if (nextPosIndex < NR_OF_POSITIONS) {
    if (isNewPosition) {
      positions[nextPosIndex].x = X;
      positions[nextPosIndex].y = Y;
      positions[nextPosIndex].z = Z;
      positions[nextPosIndex].count = 1;

      nextPosIndex++;
    } else {
      positions[nextPosIndex - 1].count++;
    }
  }
}

static void moveSetPoint(float x, float y, float z) {
  // DEBUG_PRINT("Set (%d, %d, %d)\n", (int)(x * 100.0f), (int)(y * 100.0f), (int)(z * 100.0f));
  setpoint.position.x = x;
  setpoint.position.y = y;
  setpoint.position.z = z;

  commanderSetSetpoint(&setpoint, 3);
}

static void handleStateWaitPosLock() {
  if (hasLock()) {
    soundSetEffect(SND_CALIB);
    DEBUG_PRINT("Position lock OK\n");
    changeState(ST_RECORD_TRACE);
  }
}


static void handleStateRecordTrace() {
  if (startReplay) {
    nextPosIndex--;
    DEBUG_PRINT("End of recording\n");
    changeState(ST_RETRACE);
  } else {
    recordPosition();
  }
}


static void handleStateRetrace() {
  if (nextPosIndex >= 0) {
    moveSetPoint(positions[nextPosIndex].x, positions[nextPosIndex].y, positions[nextPosIndex].z);
    positions[nextPosIndex].count--;

    if (0 == positions[nextPosIndex].count) {
      nextPosIndex--;
    }
  } else {
    DEBUG_PRINT("End of retrace\n");
    changeState(ST_STOP);
  }
}


static void handleStateStop() {
  setpoint.thrust = 0;
  setpoint.mode.z = modeDisable;

  commanderSetSetpoint(&setpoint, 3);
}
