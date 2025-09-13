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

#include "mm_flow.h"
#include "log.h"
#include "bmi088.h"
#include <stdlib.h>
#include "debug.h"

// #include "mm_position.h"//包含位置测量模块的头文件，可能用于访问位置相关的定义或函数。

#define FLOW_RESOLUTION 0.1f //光流分辨率，表示测量值需要乘以 0.1 来转换为实际的运动像素。We do get the measurements in 10x the motion pixels (experimentally measured)
#define comparsionNX 1.0f
#define comparsionNY 1.0f
// TODO remove the temporary test variables (used for logging)
// 用于存储光流传感器的预测值和测量值，主要用于日志记录和调试。
static float predictedNX;//预测值x
static float predictedNY;//预测值y
static float measuredNX;//测量值x
static float measuredNY;//测量值y
//核心函数：用于将光流传感器的测量值整合到扩展卡尔曼滤波器中。
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{ //kalmanCoreData_t* this卡尔曼滤波器的状态数据； flowMeasurement_t *flow光流传感器的测量数据； Axis3f *gyro陀螺仪的角速度数据(在IMU_types下)。
  // Inclusion of flow measurements in the EKF done by two scalar updates

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 35.0;                      // [pixels] (same in x and y)光流传感器的像素数
  //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
  float thetapix = 0.71674f;// 2*sin(42/2); 光流传感器的视场角（以弧度为单位）42degree is the agnle of aperture, here we computed the corresponding ground length
  //~~~ Body rates ~~~
  // TODO check if this is feasible or if some filtering has to be done
  // 将陀螺仪的角速度从度转换为弧度
  float omegax_b = gyro->x * DEG_TO_RAD;//gyro->x为结构体内的x轴角速度
  float omegay_b = gyro->y * DEG_TO_RAD;

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ];
  //dy_g = R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ];

  //坐标转换：将机体坐标系下的速度转换到全局坐标系。
  float dx_g = this->S[KC_STATE_PX];
  float dy_g = this->S[KC_STATE_PY];
  float z_g = 0.0;  
  // Saturate elevation in prediction and correction to avoid singularities对高度z_g进行饱和处理，避免在高度接近0时出现奇异性。
  if ( this->S[KC_STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // ~~~ X velocity prediction and update ~~~
  // predicts the number of accumulated pixels in the x-direction预测x方向累计的像素数
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * this->R[2][2] / z_g) - omegay_b);//预测的 X 方向像素变化量
  measuredNX = flow->dpixelx*FLOW_RESOLUTION;//测量的 X 方向像素变化量，积累的像素值乘以分辨率转换为实际的像素变化量，也就是像素变化速度
  // get_accel_x=accel->x;//新增获取加速度计的值
  

  // derive measurement equation with respect to dx (and z?)测量方程的雅可比矩阵。说明这是EKF代码
  hx[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dx_g) / (-z_g * z_g));//测量方程对高度的偏导数
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);//测量方程对位置x的偏导数

  //First update函数更新卡尔曼滤波器状态
  
  //if (fabsf(measuredNX-predictedNX)<comparsionNX)//表示二者差值大时不进行更新
 // {
   // DEBUG_PRINT("---Successful for x---\n");
   // DEBUG_PRINT("Kalman_updata of x_error:%f. flow_x:%0.3f, IMU_acc_x:%0.3f \n",(double)(measuredNX-predictedNX),(double)measuredNX,(double)predictedNX);
   // kalmanCoreScalarUpdate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION);//(measuredNX-predictedNX)为测量残差，flow->stdDevX*FLOW_RESOLUTION为测量噪声标准差。
 // }
  //else
  //{
  //  DEBUG_PRINT("---NO UPDATE for x---\n");
  //  DEBUG_PRINT("Kalman_updata of x_error:%f. flow_x:%0.3f, IMU_acc_x:%0.3f \n",(double)(measuredNX-predictedNX),(double)measuredNX,(double)predictedNX);
 // }
  //DEBUG_PRINT("Kalman_updata of x_error:%f. flow_x:%0.3f, IMU_acc_x:%0.3f \n",(double)(measuredNX-predictedNX),(double)measuredNX,(double)predictedNX);
  kalmanCoreScalarUpdate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION);//(measuredNX-predictedNX)为测量残差，flow->stdDevX*FLOW_RESOLUTION为测量噪声标准差。
  
  // ~~~ Y velocity prediction and update ~~~
  float hy[KC_STATE_DIM] = {0};//测量方程的雅可比矩阵。
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};//测量方程的雅可比矩阵。
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * this->R[2][2] / z_g) + omegax_b);//预测的 Y 方向像素变化量
  measuredNY = flow->dpixely*FLOW_RESOLUTION;//测量的 Y 方向像素变化量
  // get_accel_y=accel->y;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z] = (Npix * flow->dt / thetapix) * ((this->R[2][2] * dy_g) / (-z_g * z_g));//测量方程对高度的偏导数
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);//测量方程对位置y的偏导数

  // Second update
  //if (fabsf(measuredNY-predictedNY)<comparsionNY)//表示二者差值大时不进行更新
  //{
   // DEBUG_PRINT("---Successful for y---\n");
    //DEBUG_PRINT("Kalman_updata of y_error: %f [done]. flow_y:%0.3f, IMU_acc_y:%0.3f \n",(double)(measuredNY-predictedNY),(double)measuredNY,(double)predictedNY);
   // kalmanCoreScalarUpdate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION);
  //}
  //else
  //{
   // DEBUG_PRINT("---NO UPDATE---\n");
   // DEBUG_PRINT("Kalman_updata of y_error: %f [done]. flow_y:%0.3f, IMU_acc_y:%0.3f \n",(double)(measuredNY-predictedNY),(double)measuredNY,(double)predictedNY);
  //}
  //DEBUG_PRINT("Kalman_updata of y_error: %f [done]. flow_y:%0.3f, IMU_acc_y:%0.3f \n",(double)(measuredNY-predictedNY),(double)measuredNY,(double)predictedNY);
  kalmanCoreScalarUpdate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION);
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)
