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
 * Utilities to simplify unit testing
 *
 */

#pragma once

// Include "arm_math.h". This header generates some warnings, especially in
// unit tests. We hide them to avoid noise.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "arm_math.h"
#pragma GCC diagnostic pop

#include "arm_math.h"
#include "cfassert.h"


#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
    ASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst));
}

static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
    ASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
}

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) {
    ASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst));
}

static inline float arm_sqrt(float32_t in) {
    float pOut = 0;
    arm_status result = arm_sqrt_f32(in, &pOut);
    ASSERT(ARM_MATH_SUCCESS == result);
    return pOut;
}

static inline float limPos(float in) {
  if (in < 0.0f) {
    return 0.0f;
  }

  return in;
}

static inline float clip1(float a) {
  if (a < -1.0f) {
    return -1.0f;
  }

  if (a > 1.0f) {
    return 1.0f;
  }

  return a;
}
