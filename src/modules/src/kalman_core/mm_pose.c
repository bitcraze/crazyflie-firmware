/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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

#include "mm_pose.h"
#include "math3d.h"

void kalmanCoreUpdateWithPose(kalmanCoreData_t* this, poseMeasurement_t *pose)
{
  // a direct measurement of states x, y, and z, and orientation
  // do a scalar update for each state, since this should be faster than updating all together
  for (int i=0; i<3; i++) {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_X+i] = 1;
    kalmanCoreScalarUpdate(this, &H, pose->pos[i] - this->S[KC_STATE_X+i], pose->stdDevPos);
  }

  // compute orientation error
  struct quat const q_ekf = mkquat(this->q[1], this->q[2], this->q[3], this->q[0]);
  struct quat const q_measured = mkquat(pose->quat.x, pose->quat.y, pose->quat.z, pose->quat.w);
  struct quat const q_residual = qqmul(qinv(q_ekf), q_measured);
  // small angle approximation, see eq. 141 in http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
  struct vec const err_quat = vscl(2.0f / q_residual.w, quatimagpart(q_residual));

  // do a scalar update for each state
  {
    float h[KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    h[KC_STATE_D0] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.x, pose->stdDevQuat);
    h[KC_STATE_D0] = 0;

    h[KC_STATE_D1] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.y, pose->stdDevQuat);
    h[KC_STATE_D1] = 0;

    h[KC_STATE_D2] = 1;
    kalmanCoreScalarUpdate(this, &H, err_quat.z, pose->stdDevQuat);
  }
}
