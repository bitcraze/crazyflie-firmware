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

#include "cfassert.h"


#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define MAX(a, b) ((b) > (a) ? (b) : (a))

// Matrix data must be aligned on 4 byte bundaries
static inline void assert_aligned_4_bytes(const arm_matrix_instance_f32* matrix) {
  const uint32_t address = (uint32_t)matrix->pData;
  ASSERT((address & 0x3) == 0);
}

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  assert_aligned_4_bytes(pSrc);
  assert_aligned_4_bytes(pDst);

  ASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst));
}

static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst) {
  assert_aligned_4_bytes(pSrc);
  assert_aligned_4_bytes(pDst);

  ASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
}

static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst) {
  assert_aligned_4_bytes(pSrcA);
  assert_aligned_4_bytes(pSrcB);
  assert_aligned_4_bytes(pDst);

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

static inline void mat_scale(const arm_matrix_instance_f32 * pSrcA, float32_t scale, arm_matrix_instance_f32 * pDst)
{ ASSERT(ARM_MATH_SUCCESS == arm_mat_scale_f32(pSrcA, scale, pDst)); }

// copy float matrix
static inline void matrixcopy(int ROW, int COLUMN, float destmat[ROW][COLUMN], float srcmat[ROW][COLUMN]){
    //TODO: check the dimension of the matrices
    for (int i=0; i<ROW; i++){
        for(int j=0; j<COLUMN; j++){
            destmat[i][j] = srcmat[i][j];
        }
    }
}

// copy float vector
static inline void vectorcopy(int DIM, float destVec[DIM], float srcVec[DIM]){
    //TODO: check the dimension of the vector
    for (int i=0; i<DIM; i++){
        destVec[i] = srcVec[i];
    }
}