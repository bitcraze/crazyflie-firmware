/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.c - implementation of the PID regulator
 */

#include "pid.h"

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->integ += pid->error * pid->dt;
    if (pid->integ > pid->iLimit)
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < pid->iLimitLow)
    {
        pid->integ = pid->iLimitLow;
    }

    pid->deriv = (pid->error - pid->prevError) / pid->dt;

    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integ;
    pid->outD = pid->kd * pid->deriv;

    output = pid->outP + pid->outI + pid->outD;

    pid->prevError = pid->error;

    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) {
    pid->iLimitLow = limitLow;
}

void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001 && pid->ki < 0.0001 && pid->kd < 0.0001)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}
