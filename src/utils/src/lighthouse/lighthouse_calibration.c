/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse_calibration.c: lighthouse positioning angle calibration
 */

#include "lighthouse_calibration.h"

#include <math.h>
#include "cf_math.h"
#include "physicalConstants.h"

void lighthouseCalibrationInitFromFrame(lighthouseCalibration_t *calib, struct ootxDataFrame_s *frame)
{
  calib->sweep[0].phase = frame->phase0;
  calib->sweep[0].tilt = frame->tilt0;
  calib->sweep[0].curve = frame->curve0;
  calib->sweep[0].gibmag = frame->gibmag0;
  calib->sweep[0].gibphase = frame->gibphase0;
  calib->sweep[0].ogeemag = frame->ogeemag0;
  calib->sweep[0].ogeephase = frame->ogeephase0;

  calib->sweep[1].phase = frame->phase1;
  calib->sweep[1].tilt = frame->tilt1;
  calib->sweep[1].curve = frame->curve1;
  calib->sweep[1].gibmag = frame->gibmag1;
  calib->sweep[1].gibphase = frame->gibphase1;
  calib->sweep[1].ogeemag = frame->ogeemag1;
  calib->sweep[1].ogeephase = frame->ogeephase1;

  calib->uid = frame->id;
  calib->valid = true;
}

static void idealToDistortedV1(const lighthouseCalibration_t* calib, const float* ideal, float* distorted) {
  const float ax = ideal[0];
  const float ay = ideal[1];

  const float x = 1.0f;
  const float y = tanf(ax);
  const float z = tanf(ay);
  const float tIgnore = 0.0f;

  distorted[0] = lighthouseCalibrationMeasurementModelLh1(x, y, z, tIgnore, &calib->sweep[0]);
  distorted[1] = lighthouseCalibrationMeasurementModelLh1(x, z, -y, tIgnore, &calib->sweep[1]);
}

static void idealToDistortedV2(const lighthouseCalibration_t* calib, const float* ideal, float* distorted) {
  const float t30 = M_PI_F / 6.0f;
  const float tan30 = 0.5773502691896258;  // const float tan30 = tanf(t30);

  const float a1 = ideal[0];
  const float a2 = ideal[1];

  const float x = 1.0f;
  const float y = tanf((a2 + a1) / 2.0f);
  const float z = sinf(a2 - a1) / (tan30 * (cosf(a2) + cosf(a1)));

  distorted[0] = lighthouseCalibrationMeasurementModelLh2(x, y, z, -t30, &calib->sweep[0]);
  distorted[1] = lighthouseCalibrationMeasurementModelLh2(x, y, z, t30, &calib->sweep[1]);
}

typedef void (* idealToDistortedFcn_t)(const lighthouseCalibration_t* calib, const float* ideal, float* distorted);

static void lighthouseCalibrationApply(const lighthouseCalibration_t* calib, const float* rawAngles, float* correctedAngles, idealToDistortedFcn_t idealToDistorted) {
  const double max_delta = 0.0005;

  // Use distorted angle as a starting point
  float* estmatedAngles = correctedAngles;
  estmatedAngles[0] = rawAngles[0];
  estmatedAngles[1] = rawAngles[1];

  for (int i = 0; i < 5; i++) {
    float currentDistortedAngles[2];
    idealToDistorted(calib, estmatedAngles, currentDistortedAngles);

    const float delta0 = rawAngles[0] - currentDistortedAngles[0];
    const float delta1 = rawAngles[1] - currentDistortedAngles[1];

    estmatedAngles[0] = estmatedAngles[0] + delta0;
    estmatedAngles[1] = estmatedAngles[1] + delta1;

    if (fabs((double)delta0) < max_delta && fabs((double)delta1) < max_delta) {
      break;
    }
  }
}

void lighthouseCalibrationApplyV1(const lighthouseCalibration_t* calib, const float* rawAngles, float* correctedAngles) {
  return lighthouseCalibrationApply(calib, rawAngles, correctedAngles, idealToDistortedV1);
}

void lighthouseCalibrationApplyV2(const lighthouseCalibration_t* calib, const float* rawAngles, float* correctedAngles) {
  return lighthouseCalibrationApply(calib, rawAngles, correctedAngles, idealToDistortedV2);
}

void lighthouseCalibrationApplyNothing(const float rawAngles[2], float correctedAngles[2]) {
  correctedAngles[0] = rawAngles[0];
  correctedAngles[1] = rawAngles[1];
}

float lighthouseCalibrationMeasurementModelLh1(const float x, const float y, const float z, const float t, const lighthouseCalibrationSweep_t* calib) {
  const float ax = atan2f(y, x);
  const float ay = atan2f(z, x);
  const float r = arm_sqrt(x * x + y * y);

  const float compTilt = asinf(clip1(z * tanf(calib->tilt) / r));
  const float compGib = -calib->gibmag * arm_sin_f32(ax + calib->gibphase);
  const float compCurve = calib->curve * ay * ay;

  return ax - (compTilt + calib->phase + compGib + compCurve);
}

float lighthouseCalibrationMeasurementModelLh2(const float x, const float y, const float z, const float t, const lighthouseCalibrationSweep_t* calib) {
  const float ax = atan2f(y, x);
  // const float ay = atan2f(z, x);
  const float r = arm_sqrt(x * x + y * y);

  const float base = ax + asinf(clip1(z * tanf(t - calib->tilt) / r));
  const float compGib = -calib->gibmag * arm_cos_f32(ax + calib->gibphase);
  // TODO krri Figure out how to use curve and ogee calibration parameters

  return base - (calib->phase + compGib);
}
