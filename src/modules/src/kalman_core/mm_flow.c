#include "mm_flow.h"
#include "log.h"
#include "bmi088.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "debug.h"
#define USE_QUALITY_DOWNWEIGHT 1
#define Z_LOW_TH             0.20f
#define OMEGA_SUM_TH_RAD     (DEG_TO_RAD*120.0f)

// ---- 可调参数（建议先用默认值飞起来，再微调）----
#define FLOW_RESOLUTION     0.1f     // 传感器给出dpixel*需乘0.1换成实际像素数
#define NPIX_PER_AXIS       35.0f    // 视为x/y相同的“等效像素”
#define THETA_PIX_RAD       0.71674f // 约 2*sin(42deg/2)，与视场相关
#define Z_MIN_SAT           0.10f    // 预测/更新时对高度做饱和，避免奇异
#define DZ_MAX_PER_UPDATE   0.008f   // 单次光流融合引起的 Z 改变量上限（1.5 cm）
// 鲁棒融合参数
#define HUBER_DELTA         0.8f     // Huber 钝化阈值（像素/帧）
#define CHI2_GATE           9.0f     // 卡方门限 ~ 3σ
#define R_INFLATE_FACTOR    9.0f     // 异常时把测量噪声放大倍数

// 如需基于光流质量或运动/高度退化进一步降权，可启用此块
#define USE_QUALITY_DOWNWEIGHT 1
#define QUALITY_TH           10      // 示例阈值：质量过低时降权（按你的传感器/驱动定义调整）
#define OMEGA_SUM_TH_RAD     (DEG_TO_RAD*120.0f) // 旋转过大时降权
#define Z_LOW_TH             0.20f

// --------- 内部静态变量（仅用于日志）---------
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

// --------- 工具函数：计算创新方差 S = H P H^T + Rvar ----------
// 说明：若拿不到P（不同固件版本结构可能不同），则回退为 S≈Rvar，这是保守但稳定的做法。
static float kc_innovVar_orApprox(const kalmanCoreData_t* kc,
                                  const arm_matrix_instance_f32* H,
                                  float Rvar /*测量方差*/)
{
  float S = Rvar;

  // 若你的 kalmanCoreData_t 里有 P[KC_STATE_DIM][KC_STATE_DIM]，可用下列实现精确计算
  // 没有就保留默认返回 Rvar（上面的赋值已保证）
  #if defined(KC_STATE_DIM)
  if (kc && H && H->pData) {
    float v = 0.0f;
    for (int i = 0; i < KC_STATE_DIM; i++) {
      const float Hi = H->pData[i];       // H: 1xN
      if (Hi != 0.0f) {
        float sum = 0.0f;
        for (int j = 0; j < KC_STATE_DIM; j++) {
          sum += kc->P[i][j] * H->pData[j];  // (P * H^T)[i]
        }
        v += Hi * sum; // H * P * H^T
      }
    }
    S = v + Rvar;
    if (!(S > 0.0f)) S = Rvar; // 防NaN/非正
  }
  #endif

  return S;
}

// --------- 主函数：鲁棒光流融合（方案A）---------
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this,
                              const flowMeasurement_t *flow,
                              const Axis3f *gyro)
{
  // ~~~ 角速度（弧度/秒） ~~~
  const float omegax_b = gyro->x * DEG_TO_RAD;
  const float omegay_b = gyro->y * DEG_TO_RAD;

  // ~~~ 取状态里的速度（世界系），并做高度饱和 ~~~
  const float dx_g = this->S[KC_STATE_PX];
  const float dy_g = this->S[KC_STATE_PY];
  const float z_g  = (this->S[KC_STATE_Z] < Z_MIN_SAT) ? Z_MIN_SAT : this->S[KC_STATE_Z];

  // ~~~ 预测项（像素/帧）~~~
  // 注意：dpixel 为“累计像素”，乘 FLOW_RESOLUTION 转为实际像素；下面 predictedN* 也在像素尺度
  predictedNX = (flow->dt * NPIX_PER_AXIS / THETA_PIX_RAD) * ((dx_g * this->R[2][2] / z_g) - omegay_b);
  predictedNY = (flow->dt * NPIX_PER_AXIS / THETA_PIX_RAD) * ((dy_g * this->R[2][2] / z_g) + omegax_b);

  measuredNX  = flow->dpixelx * FLOW_RESOLUTION;
  measuredNY  = flow->dpixely * FLOW_RESOLUTION;

  // ~~~ X 轴：构建雅可比 Hx（1xN）~~~
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  hx[KC_STATE_Z]  = (NPIX_PER_AXIS * flow->dt / THETA_PIX_RAD) * ((this->R[2][2] * dx_g) / (-z_g * z_g));
  hx[KC_STATE_PX] = (NPIX_PER_AXIS * flow->dt / THETA_PIX_RAD) * ( this->R[2][2] /  z_g);

  // ~~~ Y 轴：构建雅可比 Hy（1xN）~~~
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  hy[KC_STATE_Z]  = (NPIX_PER_AXIS * flow->dt / THETA_PIX_RAD) * ((this->R[2][2] * dy_g) / (-z_g * z_g));
  hy[KC_STATE_PY] = (NPIX_PER_AXIS * flow->dt / THETA_PIX_RAD) * ( this->R[2][2] /  z_g);

  // ~~~ 基础测量标准差（像素/帧），注意 kalmanCoreScalarUpdate 期望传“标准差”，不是方差 ~~~
  float Rsd_x = flow->stdDevX * FLOW_RESOLUTION;
  float Rsd_y = flow->stdDevY * FLOW_RESOLUTION;

#if USE_QUALITY_DOWNWEIGHT
  // 质量字段在 flowMeasurement_t 中不存在：仅用 z 与 |ω| 做降权
  const float omega_sum = fabsf(omegax_b) + fabsf(omegay_b);
  const bool poorFusion = (z_g < Z_LOW_TH) || (omega_sum > OMEGA_SUM_TH_RAD);

  if (poorFusion) {
    Rsd_x *= 4.0f;   // 放大测量标准差 → 降低权重
    Rsd_y *= 4.0f;
  }
#endif

  // ---------------- X 轴更新：卡方门限 + Huber + 噪声膨胀 ----------------
  {
    float rX = measuredNX - predictedNX;   // 创新量
    const float Rvar_x = Rsd_x * Rsd_x;    // 测量方差
    float Sx = kc_innovVar_orApprox(this, &Hx, Rvar_x);

    // 卡方门限（极端异常 → 膨胀噪声）
    if (rX * rX > CHI2_GATE * Sx) {
      Rsd_x *= sqrtf(R_INFLATE_FACTOR);    // 方差乘以R_INFLATE_FACTOR，对应标准差乘以 sqrt(...)
      // 重新计算 Sx（可选；不重算也行，影响很小）
      const float Rvar_x2 = Rsd_x * Rsd_x;
      Sx = kc_innovVar_orApprox(this, &Hx, Rvar_x2);
    }

    // Huber 钝化：限制单次校正力度
    const float abs_rX = fabsf(rX);
    if (abs_rX > HUBER_DELTA) {
      rX = HUBER_DELTA * (rX > 0.0f ? 1.0f : -1.0f);
    }
      // --------- Z 修正限幅（保留 Z 耦合，但限制单次对 Z 的影响）---------
    #if defined(KC_STATE_DIM)
    // 计算 Kz = (P_rowZ · H^T) / Sx
    float numZx = 0.0f;
    for (int j = 0; j < KC_STATE_DIM; j++) {
      numZx += this->P[KC_STATE_Z][j] * Hx.pData[j];
    }
    float Kz_x = numZx / Sx;
    float dz_x = Kz_x * rX;  // 这次更新对 Z 的投影修正量（米）

    if (fabsf(dz_x) > DZ_MAX_PER_UPDATE && Hx.pData[KC_STATE_Z] != 0.0f) {
      // 需要按比例降低 Z 耦合项，保证 |dz_x| 不超过上限
      const float scale = DZ_MAX_PER_UPDATE / fabsf(dz_x);
      Hx.pData[KC_STATE_Z] *= scale;

      // 可选：Sx 轻度重算，提升一致性（不重算也能工作）
      Sx = kc_innovVar_orApprox(this, &Hx, Rsd_x * Rsd_x);

      // 重新计算 Kz_x·r（不是必须，只是用于调试/记录）
      // numZx = 0.0f;
      // for (int j = 0; j < KC_STATE_DIM; j++) numZx += this->P[KC_STATE_Z][j] * Hx.pData[j];
      // Kz_x = numZx / Sx;
      // dz_x = Kz_x * rX;
    }
    #endif
    // 调用核心更新（不再做不对称/跳变处理）
    kalmanCoreScalarUpdate(this, &Hx, rX, Rsd_x);
  }

  // ---------------- Y 轴更新：同上 ----------------
  {
    float rY = measuredNY - predictedNY;
    const float Rvar_y = Rsd_y * Rsd_y;
    float Sy = kc_innovVar_orApprox(this, &Hy, Rvar_y);

    if (rY * rY > CHI2_GATE * Sy) {
      Rsd_y *= sqrtf(R_INFLATE_FACTOR);
      const float Rvar_y2 = Rsd_y * Rsd_y;
      Sy = kc_innovVar_orApprox(this, &Hy, Rvar_y2);
    }

    const float abs_rY = fabsf(rY);
    if (abs_rY > HUBER_DELTA) {
      rY = HUBER_DELTA * (rY > 0.0f ? 1.0f : -1.0f);
    }
      // --------- Z 修正限幅（保留 Z 耦合，但限制单次对 Z 的影响）---------
    #if defined(KC_STATE_DIM)
    float numZy = 0.0f;
    for (int j = 0; j < KC_STATE_DIM; j++) {
      numZy += this->P[KC_STATE_Z][j] * Hy.pData[j];
    }
    float Kz_y = numZy / Sy;
    float dz_y = Kz_y * rY;

    if (fabsf(dz_y) > DZ_MAX_PER_UPDATE && Hy.pData[KC_STATE_Z] != 0.0f) {
      const float scale = DZ_MAX_PER_UPDATE / fabsf(dz_y);
      Hy.pData[KC_STATE_Z] *= scale;

      // 可选：重算 Sy
      Sy = kc_innovVar_orApprox(this, &Hy, Rsd_y * Rsd_y);
    }
    #endif
    kalmanCoreScalarUpdate(this, &Hy, rY, Rsd_y);
  }
}

// -------- 日志：保持你原来的分组 --------
LOG_GROUP_START(kalman_pred)
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)
