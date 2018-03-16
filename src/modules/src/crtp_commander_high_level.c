/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/*
High-level commander: computes smooth setpoints based on high-level inputs
such as: take-off, landing, polynomial trajectories.
*/

#include <string.h>
#include <errno.h>
#include <math.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Crazyswarm includes
#include "crtp.h"
#include "crtp_commander_high_level.h"
#include "debug.h"
#include "planner.h"
#include "log.h"
#include "param.h"

// Global variables
static bool isInit = false;
static struct planner planner;
static uint8_t group_mask;
static struct vec pos; // last known state (position [m])
static float yaw; // last known state (yaw [rad])

// makes sure that we don't evaluate the trajectory while it is being changed
static xSemaphoreHandle lockTraj;

// CRTP Packet definitions

// trajectory command (first byte of crtp packet)
enum TrajectoryCommand_e {
  COMMAND_SET_GROUP_MASK          = 0,
  COMMAND_TAKEOFF                 = 1,
  COMMAND_LAND                    = 2,
  COMMAND_STOP                    = 3,
  COMMAND_GO_TO                   = 4,
};

struct data_set_group_mask {
  uint8_t groupMask; // mask for which groups this CF belongs to
} __attribute__((packed));

// vertical takeoff from current x-y position to given height
struct data_takeoff {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// vertical land from current x-y position to given height
struct data_land {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

// stops the current trajectory (turns off the motors)
struct data_stop {
  uint8_t groupMask;        // mask for which CFs this should apply to
} __attribute__((packed));

// "take this much time to go here, then hover"
struct data_go_to {
  uint8_t groupMask; // mask for which CFs this should apply to
  uint8_t relative;  // set to true, if position/yaw are relative to current setpoint
  float x; // m
  float y; // m
  float z; // m
  float yaw; // deg
  float duration; // sec
} __attribute__((packed));

// Private functions
static void crtpCommanderHighLevelTask(void * prm);

static int set_group_mask(const struct data_set_group_mask* data);
static int takeoff(const struct data_takeoff* data);
static int land(const struct data_land* data);
static int stop(const struct data_stop* data);
static int go_to(const struct data_go_to* data);

// Helper functions
static struct vec state2vec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}

bool isInGroup(uint8_t g) {
  return g == 0 || (g & group_mask) != 0;
}

void crtpCommanderHighLevelInit(void)
{
  if (isInit) {
    return;
  }

  plan_init(&planner);

  //Start the trajectory task
  xTaskCreate(crtpCommanderHighLevelTask, CMD_HIGH_LEVEL_TASK_NAME,
              CMD_HIGH_LEVEL_TASK_STACKSIZE, NULL, CMD_HIGH_LEVEL_TASK_PRI, NULL);

  lockTraj = xSemaphoreCreateMutex();

  pos = vzero();
  yaw = 0;

  isInit = true;
  DEBUG_PRINT("traj. initialized.\n");
}

void crtpCommanderHighLevelStop()
{
  plan_stop(&planner);
}

bool crtpCommanderHighLevelIsStopped()
{
  return plan_is_stopped(&planner);
}

void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state)
{
  pos = state2vec(state->position);
  yaw = radians(state->attitude.yaw);

  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
  struct traj_eval ev = plan_current_goal(&planner, t);
  if (!is_traj_eval_valid(&ev)) {
    // programming error
    plan_stop(&planner);
  }
  xSemaphoreGive(lockTraj);

  if (is_traj_eval_valid(&ev)) {
    setpoint->position.x = ev.pos.x;
    setpoint->position.y = ev.pos.y;
    setpoint->position.z = ev.pos.z;
    setpoint->velocity.x = ev.vel.x;
    setpoint->velocity.y = ev.vel.y;
    setpoint->velocity.z = ev.vel.z;
    setpoint->attitude.yaw = degrees(ev.yaw);
    setpoint->attitudeRate.roll = degrees(ev.omega.x);
    setpoint->attitudeRate.pitch = degrees(ev.omega.y);
    setpoint->attitudeRate.yaw = degrees(ev.omega.z);
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;
    setpoint->mode.quat = modeDisable;
    setpoint->acceleration.x = ev.acc.x;
    setpoint->acceleration.y = ev.acc.y;
    setpoint->acceleration.z = ev.acc.z;
  }
}

void crtpCommanderHighLevelTask(void * prm)
{
  int ret;
  CRTPPacket p;
  crtpInitTaskQueue(CRTP_PORT_SETPOINT_HL);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_SETPOINT_HL, &p);

    switch(p.data[0])
    {
      case COMMAND_SET_GROUP_MASK:
        ret = set_group_mask((const struct data_set_group_mask*)&p.data[1]);
        break;
      case COMMAND_TAKEOFF:
        ret = takeoff((const struct data_takeoff*)&p.data[1]);
        break;
      case COMMAND_LAND:
        ret = land((const struct data_land*)&p.data[1]);
        break;
      case COMMAND_STOP:
        ret = stop((const struct data_stop*)&p.data[1]);
        break;
      case COMMAND_GO_TO:
        ret = go_to((const struct data_go_to*)&p.data[1]);
        break;
      default:
        ret = ENOEXEC;
        break;
    }

    //answer
    p.data[3] = ret;
    p.size = 4;
    crtpSendPacket(&p);
  }
}

int set_group_mask(const struct data_set_group_mask* data)
{
  group_mask = data->groupMask;

  return 0;
}

int takeoff(const struct data_takeoff* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_takeoff(&planner, pos, yaw, data->height, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int land(const struct data_land* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_land(&planner, pos, yaw, data->height, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int stop(const struct data_stop* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    plan_stop(&planner);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int go_to(const struct data_go_to* data)
{
  int result = 0;
  if (isInGroup(data->groupMask)) {
    struct vec hover_pos = mkvec(data->x, data->y, data->z);
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_go_to(&planner, data->relative, hover_pos, data->yaw, data->duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}
