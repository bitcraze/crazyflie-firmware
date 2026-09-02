/**
 * tdoa_flight.c - Link-free autonomous flight for TDoA candidate capture.
 *
 * A radio link degrades Loco performance, so a clean TDoA capture has to fly
 * with the radio off. This app makes that possible: set tdoaFlight.start over a
 * brief connection, disconnect, and after tdoaFlight.delay seconds it arms and
 * flies the sequence selected by tdoaFlight.seq, then lands and disarms.
 *
 * DELIBERATELY NOT IN SCOPE: starting the log. usd.logging and
 * tdoaEngine.logCand stay under script control so that the capture tooling
 * (tools/usdlog/) keeps working against any firmware, with or without this app.
 * The one exception is stopping the log at touchdown -- see stopLog below.
 *
 * Why the app has to do the arming, rather than a script arming before it
 * disconnects: PREFLIGHT_TIMEOUT_MS is 30000 (platform/interface/
 * platform_defaults.h) and supervisorIsPreflightTimeout() measures from the
 * arming tick, so an armed drone that has not taken off within 30 s falls out
 * of supervisorStateReadyToFly and supervisor.c disarms it. Arming has to
 * happen immediately before takeoff, which is after the link is gone.
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "crtp_commander_high_level.h"
#include "eventtrigger.h"
#include "log.h"
#include "param.h"
#include "pm.h"
#include "supervisor.h"

#define DEBUG_MODULE "TDOAFLIGHT"
#include "debug.h"

#include "flight_sequences.h"

// --- Tuning that is not worth a param ---------------------------------------

#define TICK_MS              50
#define SETTLE_MS            3000    // let the system come up before touching params

#define LOCK_SAMPLES         40      // 2 s of estimator variance at TICK_MS
#define LOCK_TIMEOUT_MS      40000

#define ARM_TIMEOUT_MS       8000    // arming request -> supervisorCanFly()
                                     // (ARMING_SPINUP_TIMEOUT_MS is 2000)
#define TAKEOFF_HEIGHT_M     0.6f
#define TAKEOFF_DURATION_S   3.0f
#define TAKEOFF_TIMEOUT_MS   10000
#define LAND_HEIGHT_M        0.05f
#define LAND_DURATION_S      3.5f
#define LAND_TIMEOUT_MS      15000

#define STEP_SLACK_MS        5000    // grace on top of a step's nominal duration
#define FLIGHT_TIMEOUT_MS    420000  // hard cap on a whole run

#define RANDOM_WALK_STEPS    25

// --- Observable state -------------------------------------------------------

typedef enum {
  flightIdle        = 0,
  flightCountdown   = 1,
  flightWaitForLock = 2,
  flightArming      = 3,
  flightTakingOff   = 4,
  flightRunning     = 5,
  flightLanding     = 6,
  flightDone        = 7,
  flightAborted     = 8,
} flightState_t;

typedef enum {
  abortNone            = 0,
  abortNoEstimatorLock = 1,
  abortCannotArm       = 2,
  abortGeofence        = 3,
  abortTumbled         = 4,
  abortBatteryLow      = 5,
  abortCommandRejected = 6,
  abortStepTimeout     = 7,
  abortFlightTimeout   = 8,
  abortLoggingNotReady = 9,
  abortBadSequence     = 10,
  abortRequested       = 11,
  abortTakeoffFailed   = 12,
} abortReason_t;

static uint8_t stateLog      = flightIdle;
static uint8_t abortLog      = abortNone;
static uint8_t seqLog        = 0;
static uint8_t stepLog       = 0;
static float   lockSpreadLog = 0.0f;   // live peak-to-peak, for tuning pLockThr

/* Emitted on every state or step change, which is a few dozen times in a whole
 * flight. This is deliberately an event and not a periodic log variable: the
 * values change a handful of times per run, so sampling them alongside the
 * 50 Hz block would spend ring buffer bandwidth -- the resource whose overrun
 * silently puts holes in the capture -- on repeating what is already known.
 *
 * It is also strictly more informative: the event carries the exact timestamp
 * of each transition, so the log can be segmented by flight step, which
 * periodic sampling only approximates to within a sample period. */
EVENTTRIGGER(tdoaFlightStep, uint8, seq, uint8, step, uint8, state, uint8, abortR)

static void emitFlightEvent(void)
{
  eventTrigger_tdoaFlightStep_payload.seq = seqLog;
  eventTrigger_tdoaFlightStep_payload.step = stepLog;
  eventTrigger_tdoaFlightStep_payload.state = stateLog;
  eventTrigger_tdoaFlightStep_payload.abortR = abortLog;
  eventTrigger(&eventTrigger_tdoaFlightStep);
}

// --- Params -----------------------------------------------------------------

static uint8_t  pStart    = 0;      // write 1 to trigger; cleared on accept
static uint8_t  pAbort    = 0;      // write 1 to land now
static uint8_t  pSeq      = 0;
static uint8_t  pDelayS   = 10;
static uint8_t  pStopLog  = 1;      // clear usd.logging at touchdown
static uint8_t  pReqLog   = 1;      // refuse to fly if usd.canLog is 0
static uint16_t pSeed     = 1;
static uint16_t pArmWaitMs = 1500;  // settle time after canFly, before takeoff

/* Largest peak-to-peak movement allowed in any Kalman position variance over
 * the LOCK_SAMPLES window before takeoff.
 *
 * Sized for TDoA, which is much noisier than the 0.001 the Bitcraze demos use
 * against Lighthouse and Flow: measured spread on a resting drone in this rig
 * was ~0.0125 on varPX over 2 s, so anything near 0.001 never passes. Watch
 * tdoaFlight.lockSpr to see the live value and pick a number for your system. */
static float pLockThr = 0.05f;

static float pFenceXY = 2.5f;
static float pFenceZ  = 2.5f;
static float pWorkXY  = 2.0f;
static float pZLo     = 0.5f;
static float pZHi     = 2.0f;

// --- Cached ids -------------------------------------------------------------

static logVarId_t idX, idY, idZ, idVarPX, idVarPY, idVarPZ;
static paramVarId_t idEnHighLevel, idUsdCanLog, idUsdLogging;

// --- Run state --------------------------------------------------------------

static uint32_t stateEnteredTick;
static uint32_t flightStartTick;
static uint32_t stepStartTick;
static uint32_t stepDurationMs;
static uint8_t  stepIndex;
static uint8_t  runningSeq;
static bool     landCommandIssued;
static bool     armReadySeen;
static uint32_t armReadyTick;

static float lockData[LOCK_SAMPLES][3];
static uint8_t lockCount;
static uint8_t lockWrite;

static uint32_t rngState;
static float randomLastX, randomLastY, randomLastZ;

// --- Small helpers ----------------------------------------------------------

static uint32_t msSince(const uint32_t tick)
{
  return (uint32_t)((xTaskGetTickCount() - tick) * portTICK_PERIOD_MS);
}

static void enterState(const flightState_t next)
{
  stateLog = (uint8_t)next;
  stateEnteredTick = xTaskGetTickCount();
  emitFlightEvent();
}

static float mapXY(const float n) { return n * pWorkXY; }
static float mapZ(const float n)  { return pZLo + n * (pZHi - pZLo); }

static float estX(void) { return logGetFloat(idX); }
static float estY(void) { return logGetFloat(idY); }
static float estZ(void) { return logGetFloat(idZ); }

static bool insideFence(void)
{
  return fabsf(estX()) <= pFenceXY &&
         fabsf(estY()) <= pFenceXY &&
         estZ()        <= pFenceZ;
}

static void resetLock(void)
{
  lockCount = 0;
  lockWrite = 0;
}

/**
 * The estimator has converged when the Kalman position variances have stopped
 * moving. Variance, not position: a stationary drone with a diverged filter
 * also has a still position estimate, but its variance is still shrinking.
 */
static bool hasEstimatorLock(void)
{
  lockData[lockWrite][0] = logGetFloat(idVarPX);
  lockData[lockWrite][1] = logGetFloat(idVarPY);
  lockData[lockWrite][2] = logGetFloat(idVarPZ);

  lockWrite = (uint8_t)((lockWrite + 1) % LOCK_SAMPLES);
  if (lockCount < LOCK_SAMPLES) {
    lockCount++;
    return false;
  }

  float worst = 0.0f;
  for (int axis = 0; axis < 3; axis++) {
    float lo = FLT_MAX;
    float hi = -FLT_MAX;
    for (int i = 0; i < LOCK_SAMPLES; i++) {
      lo = fminf(lo, lockData[i][axis]);
      hi = fmaxf(hi, lockData[i][axis]);
    }
    worst = fmaxf(worst, hi - lo);
  }
  lockSpreadLog = worst;
  return worst < pLockThr;
}

// --- Sequence access --------------------------------------------------------

static void rngSeed(const uint16_t seed)
{
  rngState = 0x9E3779B9u ^ (uint32_t)seed;
}

/** Uniform in [-1, 1). Plain LCG: reproducibility matters, quality does not. */
static float rngUnit(void)
{
  rngState = rngState * 1664525u + 1013904223u;
  return ((float)((rngState >> 16) & 0xFFFFu) / 32768.0f) - 1.0f;
}

/**
 * Fill *out with step `idx` of sequence `seq`. Returns false at the end of the
 * sequence. Must be called exactly once per step index: the random walk
 * advances its generator here, which is what makes a given seed replayable.
 */
static bool fetchStep(const uint8_t seq, const uint8_t idx, flightStep_t* out)
{
  if (seq == TDOA_FLIGHT_SEQ_RANDOM) {
    if (idx >= RANDOM_WALK_STEPS) {
      return false;
    }

    const float x = 0.9f * rngUnit();
    const float y = 0.9f * rngUnit();
    const float z = 0.5f + 0.4f * rngUnit();

    // Hold roughly 0.5 m/s so long hops are not flown as fast as short ones.
    const float dx = mapXY(x - randomLastX);
    const float dy = mapXY(y - randomLastY);
    const float dz = mapZ(z) - mapZ(randomLastZ);
    float duration = 2.0f * sqrtf(dx * dx + dy * dy + dz * dz);
    if (duration < 3.0f) {
      duration = 3.0f;
    }

    randomLastX = x;
    randomLastY = y;
    randomLastZ = z;

    const flightStep_t step = GOTO_NORM(x, y, z, 0.0f, duration);
    *out = step;
    return true;
  }

  if (seq >= TDOA_FLIGHT_N_TABLE_SEQ) {
    return false;
  }

  const flightStep_t step = flightSequences[seq].steps[idx];
  if (step.kind == STEP_END) {
    return false;
  }
  *out = step;
  return true;
}

/** Issue a step. Returns false if the high level commander rejected it. */
static bool startStep(const flightStep_t* step)
{
  int result = 0;

  switch (step->kind) {
    case STEP_GOTO_NORM:
      result = crtpCommanderHighLevelGoTo(mapXY(step->a), mapXY(step->b),
                                          mapZ(step->c), step->d, step->e,
                                          false);
      break;

    case STEP_GOTO_REL:
      // Already metres, and mapZ's zLo floor must not be added to a delta.
      result = crtpCommanderHighLevelGoTo(step->a, step->b, step->c,
                                          step->d, step->e, true);
      break;

    case STEP_SPIRAL_NORM:
      result = crtpCommanderHighLevelSpiral(step->a, mapXY(step->b),
                                            mapXY(step->c), step->d, step->e,
                                            step->flag);
      break;

    case STEP_HOVER:
      // Nothing to command: the previous setpoint is held for the duration.
      break;

    default:
      return false;
  }

  if (result != 0) {
    DEBUG_PRINT("step %u rejected (%d)\n", stepIndex, result);
    return false;
  }

  stepStartTick = xTaskGetTickCount();
  stepDurationMs = (uint32_t)(step->e * 1000.0f) + STEP_SLACK_MS;
  return true;
}

// --- Landing / abort --------------------------------------------------------

static void beginLanding(const abortReason_t reason)
{
  if (reason != abortNone && abortLog == abortNone) {
    abortLog = (uint8_t)reason;
  }
  landCommandIssued = false;
  enterState(flightLanding);
}

/** Cut the motors immediately. Only for cases where landing is not an option. */
static void emergencyStop(const abortReason_t reason)
{
  crtpCommanderHighLevelStop();
  supervisorRequestArming(false);
  if (abortLog == abortNone) {
    abortLog = (uint8_t)reason;
  }
  DEBUG_PRINT("emergency stop, reason %u\n", abortLog);
  enterState(flightAborted);
}

static void finishRun(void)
{
  supervisorRequestArming(false);

  /* Closing the log is data preservation, not policy: an unclosed file is
   * 0 bytes, so a battery that dies between landing and the next connection
   * would otherwise take the whole capture with it. Starting the log stays
   * with the script. */
  if (pStopLog && PARAM_VARID_IS_VALID(idUsdLogging)) {
    paramSetInt(idUsdLogging, 0);
  }

  enterState(abortLog == abortNone ? flightDone : flightAborted);
  DEBUG_PRINT("run finished, state %u, abort %u\n", stateLog, abortLog);
}

// --- In-flight safety -------------------------------------------------------

/** Returns false if the caller should stop processing this tick. */
static bool checkInFlightSafety(void)
{
  if (supervisorIsTumbled()) {
    emergencyStop(abortTumbled);
    return false;
  }

  if (!insideFence()) {
    DEBUG_PRINT("geofence breach at %.2f %.2f %.2f\n",
                (double)estX(), (double)estY(), (double)estZ());
    beginLanding(abortGeofence);
    return false;
  }

  if (pmIsBatteryLow()) {
    beginLanding(abortBatteryLow);
    return false;
  }

  if (pAbort) {
    pAbort = 0;
    beginLanding(abortRequested);
    return false;
  }

  if (msSince(flightStartTick) > FLIGHT_TIMEOUT_MS) {
    beginLanding(abortFlightTimeout);
    return false;
  }

  return true;
}

// --- State machine ----------------------------------------------------------

static void tick(void)
{
  switch ((flightState_t)stateLog) {
    case flightIdle:
      if (pStart) {
        pStart = 0;
        abortLog = abortNone;
        stepIndex = 0;
        stepLog = 0;
        runningSeq = pSeq;
        seqLog = pSeq;
        armReadySeen = false;
        DEBUG_PRINT("triggered: seq %u, arming in %u s\n", pSeq, pDelayS);
        enterState(flightCountdown);
      }
      break;

    case flightCountdown:
      if (pAbort) {
        pAbort = 0;
        DEBUG_PRINT("cancelled before arming\n");
        enterState(flightIdle);
        break;
      }
      if (msSince(stateEnteredTick) < (uint32_t)pDelayS * 1000u) {
        break;
      }

      if (runningSeq > TDOA_FLIGHT_SEQ_RANDOM) {
        abortLog = abortBadSequence;
        enterState(flightAborted);
        break;
      }

      /* usd.canLog is the only signal that the card mounted AND config.txt
       * parsed, so it is the difference between a wasted battery and a
       * capture. Refuse rather than fly a run that logs nothing. */
      if (pReqLog) {
        if (!PARAM_VARID_IS_VALID(idUsdCanLog) || paramGetInt(idUsdCanLog) == 0) {
          DEBUG_PRINT("usd.canLog is 0, refusing to fly\n");
          abortLog = abortLoggingNotReady;
          enterState(flightAborted);
          break;
        }
      }

      if (PARAM_VARID_IS_VALID(idEnHighLevel)) {
        paramSetInt(idEnHighLevel, 1);
      }

      rngSeed(pSeed);
      randomLastX = 0.0f;
      randomLastY = 0.0f;
      randomLastZ = 0.5f;

      resetLock();
      enterState(flightWaitForLock);
      break;

    case flightWaitForLock:
      if (pAbort) {
        pAbort = 0;
        DEBUG_PRINT("cancelled before arming\n");
        enterState(flightIdle);
        break;
      }
      if (hasEstimatorLock()) {
        DEBUG_PRINT("estimator locked at %.2f %.2f %.2f\n",
                    (double)estX(), (double)estY(), (double)estZ());
        enterState(flightArming);
        break;
      }
      if (msSince(stateEnteredTick) > LOCK_TIMEOUT_MS) {
        DEBUG_PRINT("no estimator lock\n");
        abortLog = abortNoEstimatorLock;
        enterState(flightAborted);
      }
      break;

    case flightArming:
      if (!supervisorIsArmed()) {
        supervisorRequestArming(true);
      }

      /* Armed is not the same as allowed to fly, and waiting for the wrong one
       * fails every takeoff. supervisorIsArmed() returns isArmingActivated,
       * which is true the instant the request above returns; the state machine
       * still has to pass the motor spinup check (ARMING_SPINUP_TIMEOUT_MS,
       * 2000) before reaching supervisorStateReadyToFly. Until it does,
       * supervisorCanFly() is false, stabilizer.c:333 holds the high level
       * commander blocked via crtpCommanderBlock(!canFly), and every takeoff
       * returns EBUSY at crtp_commander_high_level.c:498. So gate on canFly,
       * and require armed too, since canFly is also true in
       * supervisorStateLanded when nothing is armed at all. */
      if (!(supervisorIsArmed() && supervisorCanFly())) {
        armReadySeen = false;
        if (msSince(stateEnteredTick) > ARM_TIMEOUT_MS) {
          DEBUG_PRINT("could not arm (armed=%d canFly=%d)\n",
                      supervisorIsArmed(), supervisorCanFly());
          abortLog = abortCannotArm;
          enterState(flightAborted);
        }
        break;
      }

      /* canFly flips as soon as the supervisor reaches ReadyToFly, which is
       * earlier than the motors are usefully spinning -- on a brushless
       * platform the ESCs need a moment at idle before they track a thrust
       * ramp cleanly. Asking for a takeoff into that window gives a ragged
       * lift-off at best. Let them settle first. */
      if (!armReadySeen) {
        armReadySeen = true;
        armReadyTick = xTaskGetTickCount();
        DEBUG_PRINT("armed, letting motors settle for %u ms\n", pArmWaitMs);
      }
      if (msSince(armReadyTick) < (uint32_t)pArmWaitMs) {
        break;
      }

      if (crtpCommanderHighLevelTakeoff(TAKEOFF_HEIGHT_M, TAKEOFF_DURATION_S) != 0) {
        emergencyStop(abortTakeoffFailed);
        break;
      }
      flightStartTick = xTaskGetTickCount();
      DEBUG_PRINT("taking off\n");
      enterState(flightTakingOff);
      break;

    case flightTakingOff:
      if (supervisorIsTumbled()) {
        emergencyStop(abortTumbled);
        break;
      }
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        stepIndex = 0;
        stepLog = 0;
        enterState(flightRunning);
        stepDurationMs = 0;   // forces the first step to be issued below
        stepStartTick = xTaskGetTickCount();
        break;
      }
      if (msSince(stateEnteredTick) > TAKEOFF_TIMEOUT_MS) {
        beginLanding(abortTakeoffFailed);
      }
      break;

    case flightRunning: {
      if (!checkInFlightSafety()) {
        break;
      }

      const bool stepRunning = (stepDurationMs != 0);
      if (stepRunning) {
        const uint32_t elapsed = msSince(stepStartTick);
        const bool done = (elapsed >= stepDurationMs - STEP_SLACK_MS) &&
                          crtpCommanderHighLevelIsTrajectoryFinished();
        if (!done) {
          if (elapsed > stepDurationMs) {
            DEBUG_PRINT("step %u timed out\n", stepIndex);
            beginLanding(abortStepTimeout);
          }
          break;
        }
        stepIndex++;
        stepLog = stepIndex;
        emitFlightEvent();
      }

      flightStep_t step;
      if (!fetchStep(runningSeq, stepIndex, &step)) {
        DEBUG_PRINT("sequence complete\n");
        beginLanding(abortNone);
        break;
      }

      if (!startStep(&step)) {
        beginLanding(abortCommandRejected);
      }
      break;
    }

    case flightLanding:
      if (supervisorIsTumbled()) {
        emergencyStop(abortTumbled);
        break;
      }

      if (!landCommandIssued) {
        landCommandIssued = true;
        if (crtpCommanderHighLevelLand(LAND_HEIGHT_M, LAND_DURATION_S) != 0) {
          emergencyStop(abortCommandRejected);
          break;
        }
        enterState(flightLanding);   // restart the landing timeout clock
        break;
      }

      if (crtpCommanderHighLevelIsTrajectoryFinished() ||
          msSince(stateEnteredTick) > LAND_TIMEOUT_MS) {
        crtpCommanderHighLevelStop();
        finishRun();
      }
      break;

    case flightDone:
    case flightAborted:
      /* Terminal until re-triggered. Re-arming for another sequence needs no
       * link: set tdoaFlight.start again and the uSD deck opens a new
       * numbered file. */
      if (pStart) {
        enterState(flightIdle);
      }
      break;

    default:
      enterState(flightIdle);
      break;
  }
}

void appMain(void)
{
  vTaskDelay(M2T(SETTLE_MS));

  idX = logGetVarId("stateEstimate", "x");
  idY = logGetVarId("stateEstimate", "y");
  idZ = logGetVarId("stateEstimate", "z");
  idVarPX = logGetVarId("kalman", "varPX");
  idVarPY = logGetVarId("kalman", "varPY");
  idVarPZ = logGetVarId("kalman", "varPZ");

  idEnHighLevel = paramGetVarId("commander", "enHighLevel");
  idUsdCanLog = paramGetVarId("usd", "canLog");
  idUsdLogging = paramGetVarId("usd", "logging");

  if (!logVarIdIsValid(idX) || !logVarIdIsValid(idVarPX)) {
    DEBUG_PRINT("missing log variables, is the Kalman estimator built in?\n");
  }

  enterState(flightIdle);
  DEBUG_PRINT("ready: set tdoaFlight.seq then tdoaFlight.start = 1\n");

  while (true) {
    vTaskDelay(M2T(TICK_MS));
    tick();
  }
}

// --- Params and logs --------------------------------------------------------

/**
 * Autonomous flight for link-free TDoA capture.
 */
PARAM_GROUP_START(tdoaFlight)
/**
 * @brief Write 1 to start a run. Cleared back to 0 as soon as it is accepted,
 * so it reads 0 again afterwards -- watch tdoaFlight.state, not this.
 */
PARAM_ADD(PARAM_UINT8, start, &pStart)
/**
 * @brief Write 1 to land immediately. Only reachable over a link, so it is a
 * convenience for bench testing, not the in-flight safety net.
 */
PARAM_ADD(PARAM_UINT8, abort, &pAbort)
/**
 * @brief Sequence index: 0 hover, 1 box, 2 spiral, 3 vertical, 4 relative
 * range, 5 seeded random walk. Recorded in the log as tdoaFlight.seq.
 */
PARAM_ADD(PARAM_UINT8, seq, &pSeq)
/**
 * @brief Seconds between the trigger and arming. Has to cover disconnecting
 * the radio and walking clear.
 */
PARAM_ADD(PARAM_UINT8, delay, &pDelayS)
/**
 * @brief Seed for sequence 4. The same seed flies the same path, so a run that
 * produced something interesting can be repeated exactly.
 */
PARAM_ADD(PARAM_UINT16, seed, &pSeed)
/**
 * @brief Set usd.logging to 0 at touchdown (default 1). Starting the log is
 * left to the capture scripts; this only closes the file so a flat battery
 * cannot cost the whole run.
 */
PARAM_ADD(PARAM_UINT8, stopLog, &pStopLog)
/**
 * @brief Refuse to fly when usd.canLog is 0 (default 1). Set to 0 to fly a
 * sequence without the uSD deck, for testing.
 */
PARAM_ADD(PARAM_UINT8, reqLog, &pReqLog)
/**
 * @brief Geofence half-width in x and y (m). Breaching it lands the drone.
 */
PARAM_ADD(PARAM_FLOAT, fenceXY, &pFenceXY)
/**
 * @brief Geofence ceiling (m). Breaching it lands the drone.
 */
PARAM_ADD(PARAM_FLOAT, fenceZ, &pFenceZ)
/**
 * @brief Half-width of the working box (m). Sequence x/y of +-1.0 map to this,
 * so it should sit inside fenceXY with margin.
 */
PARAM_ADD(PARAM_FLOAT, workXY, &pWorkXY)
/**
 * @brief Height that sequence z = 0.0 maps to (m).
 */
PARAM_ADD(PARAM_FLOAT, zLo, &pZLo)
/**
 * @brief Height that sequence z = 1.0 maps to (m).
 */
PARAM_ADD(PARAM_FLOAT, zHi, &pZHi)
/**
 * @brief Largest peak-to-peak movement allowed in any Kalman position variance
 * over a 2 s window before takeoff. Raise it if the run never leaves state 2;
 * watch the live value in the tdoaFlight.lockSpr log variable to pick one.
 */
PARAM_ADD(PARAM_FLOAT, lockThr, &pLockThr)
/**
 * @brief Milliseconds to let the motors settle at idle after the supervisor
 * allows flight, before commanding takeoff. Brushless ESCs need a moment
 * before they track a thrust ramp cleanly.
 */
PARAM_ADD(PARAM_UINT16, armWait, &pArmWaitMs)
PARAM_GROUP_STOP(tdoaFlight)

/**
 * State of the autonomous capture flight. Add these to config.txt to have each
 * log say what it is: the uSD deck skips log variables it does not know, so the
 * same config.txt still works on firmware built without this app.
 */
LOG_GROUP_START(tdoaFlight)
/**
 * @brief 0 idle, 1 countdown, 2 waiting for estimator lock, 3 arming,
 * 4 taking off, 5 running the sequence, 6 landing, 7 done, 8 aborted.
 */
LOG_ADD(LOG_UINT8, state, &stateLog)
/**
 * @brief Why the run ended early. 0 none, 1 no estimator lock, 2 cannot arm,
 * 3 geofence, 4 tumbled, 5 battery low, 6 command rejected, 7 step timeout,
 * 8 flight timeout, 9 logging not ready, 10 bad sequence, 11 abort requested,
 * 12 takeoff failed.
 */
LOG_ADD(LOG_UINT8, abortR, &abortLog)
/**
 * @brief Sequence index this run is flying.
 */
LOG_ADD(LOG_UINT8, seq, &seqLog)
/**
 * @brief Index of the step currently being flown.
 */
LOG_ADD(LOG_UINT8, step, &stepLog)
/**
 * @brief Live peak-to-peak movement of the Kalman position variances over the
 * lock window, the value compared against tdoaFlight.lockThr. Only meaningful
 * while state is 2 (waiting for estimator lock).
 */
LOG_ADD(LOG_FLOAT, lockSpr, &lockSpreadLog)
LOG_GROUP_STOP(tdoaFlight)
