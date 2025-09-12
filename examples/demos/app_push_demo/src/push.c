// app_oa_supervisor.c
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "math.h"

#define DEBUG_MODULE "OA_SUP"

static uint8_t oaEnable = 0; // 默认关闭，避免一上电就覆盖外部
PARAM_GROUP_START(oa)
PARAM_ADD(PARAM_UINT8, enable, &oaEnable)  // cfclient: Parameters -> oa.enable = 1 开启
PARAM_GROUP_STOP(oa)

// ---------- 工具宏 ----------
#define CLAMP(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
static inline float lowpass(float y_prev, float u, float alpha) {
  // alpha in [0,1], higher = faster
  return y_prev + alpha * (u - y_prev);
}
#define RANGE_VALID_MAX_MM 2000
#define SENSOR_TIMEOUT_MS  200   // 关键方向 200ms 无更新则不接管

// ---------- 参数（单位：mm 或 m/s）----------
static const uint16_t D_FRONT_ENTER_MM = 400;   // 前向进入阈值 0.60 m
static const uint16_t D_FRONT_CLEAR_MM = 550;   // 前向释放阈值 0.80 m
static const uint16_t D_SIDE_ENTER_MM  = 300;   // 侧向进入阈值 0.50 m
static const uint16_t D_SIDE_CLEAR_MM  = 450;   // 侧向释放阈值 0.65 m
static const uint16_t D_BACK_ENTER_MM = 300;  // 
static const uint16_t D_BACK_CLEAR_MM = 450;  // 


static const uint32_t T_CLEAR_MS = 300;         // 满足释放阈值后继续观察时间（去抖）
static const float    LP_ALPHA   = 0.25f;       // 距离读数一阶低通系数（0~1）

// 速度与限幅（机体系）
static const float V_SIDE_MAX       = 0.20f;    // 侧移最大速度 m/s
static const float K_SIDE           = 1.0f;     // 侧向比例系数（m/s per m误差）
static const float V_HOLD_BACK_MAX  = 0.20f;    // 前向保持时允许的后退上限
static const float K_HOLD           = 1.0f;     // 前向保持比例（m/s per m误差）
static const float V_PASSTHROUGH_MAX= 0.30f;    // 最终速度统一限幅
static const float V_SLEW_MAX       = 2.0f;     // 侧向指令斜率限幅 m/s^2（平滑过渡）

static uint32_t lastValidMs_front = 0;
static uint32_t lastValidMs_left  = 0;
static uint32_t lastValidMs_right = 0;
static uint32_t lastValidMs_back  = 0;
// Commander 优先级（3 常用于高优先级）
static const int CMD_PRIORITY = 3;

// ---------- Setpoint 辅助 ----------
static void setVelocityBodyXY(setpoint_t* sp, float vx, float vy) {
  // 在不破坏外部任务其余通道的前提下，强制 x/y 进入速度模式（机体系）
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->velocity.x = vx;
  sp->velocity.y = vy;
  sp->velocity_body = true;
}

typedef enum {
  OA_NORMAL = 0,       // 不接管，透传（本 app 不写 setpoint）
  OA_AVOID_SIDE,       // 侧向距离保持（右近左移/左近右移）
  OA_AVOID_FRONT,      // 前向保持（禁止前进，可轻微后退）
  OA_AVOID_REAR,
  OA_CLEARING          // 清空状态：刚满足释放条件，短暂观察后回到 NORMAL
} oa_state_t;

static oa_state_t state = OA_NORMAL;
static uint32_t clearStartMs = 0;
static int sideSign = 0; // +1: 右侧近（向左/Y+），-1: 左侧近（向右/Y-）

// 斜率限幅
static float slewLimit(float prev, float target, float dt, float rate) {
  float dv = target - prev;
  float maxStep = rate * dt;
  if (dv >  maxStep) return prev + maxStep;
  if (dv < -maxStep) return prev - maxStep;
  return target;
}

void appMain(void) {
  // 日志变量（MultiRanger，单位通常为 mm，>2m 可能钳制或无效）
  //logVarId_t idUp    = logGetVarId("range", "up");
  logVarId_t idLeft  = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack  = logGetVarId("range", "back");

  // 可选：检查 deck 是否已初始化
  //paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger     = paramGetVarId("deck", "bcMultiranger");

  // 初始延时，等待系统稳定
  vTaskDelay(M2T(1500));
  DEBUG_PRINT("OA Supervisor starting...\n");

  // IIR 滤波初值
  float f_mm=2000, l_mm=2000, r_mm=2000, b_mm=2000;
  // 侧向速度平滑
  float vy_cmd_prev = 0.0f;

  //const float dt = 0.010f; // 10 ms → 100 Hz
  uint32_t lastTick = xTaskGetTickCount();
  
  for (;;) {
    vTaskDelay(M2T(10));
    
    uint32_t nowTick = xTaskGetTickCount();
    float dt = (float)(nowTick - lastTick) * (1.0f / configTICK_RATE_HZ);
    lastTick = nowTick;
    
    // 若 deck 未就绪，直接跳过（不接管）
    //uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    // 使能开关：0 则完全透传，不接管
    if (!oaEnable) {
      state = OA_NORMAL;
      continue;
    }
    
    // 读取并低通（把 0 当作无效读数忽略）
    uint16_t f_raw = logGetUint(idFront);
    uint16_t l_raw = logGetUint(idLeft);
    uint16_t r_raw = logGetUint(idRight);
    uint16_t b_raw = logGetUint(idBack);
    
    // ... 读取 raw 后，做钳制 & 低通 & 更新时间戳 ...
    uint32_t nowMs = nowTick * portTICK_PERIOD_MS;

    if (f_raw > 0) {
      if (f_raw > RANGE_VALID_MAX_MM) f_raw = RANGE_VALID_MAX_MM;
      f_mm = lowpass(f_mm, (float)f_raw, LP_ALPHA);
      lastValidMs_front = nowMs;
    }
    if (l_raw > 0) {
      if (l_raw > RANGE_VALID_MAX_MM) l_raw = RANGE_VALID_MAX_MM;
      l_mm = lowpass(l_mm, (float)l_raw, LP_ALPHA);
      lastValidMs_left = nowMs;
    }
    if (r_raw > 0) {
      if (r_raw > RANGE_VALID_MAX_MM) r_raw = RANGE_VALID_MAX_MM;
      r_mm = lowpass(r_mm, (float)r_raw, LP_ALPHA);
      lastValidMs_right = nowMs;
    }
    
    if (b_raw > 0) {
  	if (b_raw > RANGE_VALID_MAX_MM) b_raw = RANGE_VALID_MAX_MM;
  	b_mm = lowpass(b_mm, (float)b_raw, LP_ALPHA);
  	lastValidMs_back = nowMs;     // 新增：后向有效更新时间
    }
    
    // 关键方向传感器超时：不接管
    bool frontOk = (nowMs - lastValidMs_front) <= SENSOR_TIMEOUT_MS;
    bool leftOk  = (nowMs - lastValidMs_left ) <= SENSOR_TIMEOUT_MS;
    bool rightOk = (nowMs - lastValidMs_right) <= SENSOR_TIMEOUT_MS;
    bool backOk  = (nowMs - lastValidMs_back ) <= SENSOR_TIMEOUT_MS;
    
    if (!multirangerInit || (!frontOk && !leftOk && !rightOk && !backOk)) {
      state = OA_NORMAL;
      continue;
    }
    
    // 状态机：进入条件
    if (state == OA_NORMAL) {
      if (f_mm < D_FRONT_ENTER_MM) {
        state = OA_AVOID_FRONT;
        clearStartMs = 0;
      } else if (b_mm < D_BACK_ENTER_MM) {
        state = OA_AVOID_REAR;  clearStartMs = 0;    // ← 新增
      } else if (r_mm < D_SIDE_ENTER_MM) {
        state = OA_AVOID_SIDE; sideSign = +1; // 右近 → 向左（Y+）
        clearStartMs = 0;
      } else if (l_mm < D_SIDE_ENTER_MM) {
        state = OA_AVOID_SIDE; sideSign = -1; // 左近 → 向右（Y-）
        clearStartMs = 0;
      }
    }

    // 前向优先
    if (state == OA_AVOID_SIDE && f_mm < D_FRONT_ENTER_MM) {
      state = OA_AVOID_FRONT;
      clearStartMs = 0;
    }

    // 释放条件去抖（满足 clear 阈值并持续 T_CLEAR_MS）
    bool readyClear = false;
    if (state == OA_AVOID_FRONT) {
      if (f_mm > D_FRONT_CLEAR_MM) readyClear = true;
    } else if (state == OA_AVOID_REAR) {                    // ← 新增
      if (b_mm > D_BACK_CLEAR_MM)  readyClear = true;
    } else if (state == OA_AVOID_SIDE) {
      float d_mm = (sideSign > 0 ? r_mm : l_mm);
      if (d_mm > D_SIDE_CLEAR_MM && f_mm > D_FRONT_ENTER_MM && b_mm > D_BACK_ENTER_MM) readyClear = true;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (state == OA_AVOID_FRONT || state == OA_AVOID_SIDE) {
      if (readyClear) {
        if (clearStartMs == 0) clearStartMs = now;
        if (now - clearStartMs >= T_CLEAR_MS) {
          state = OA_CLEARING;
          clearStartMs = 0;
        }
      } else {
        clearStartMs = 0;
      }
    } else if (state == OA_CLEARING) {
      state = OA_NORMAL;
      setpoint_t spTmp = {0};
      commanderGetSetpoint(&spTmp, NULL);
      vy_cmd_prev = spTmp.velocity.y;
    }

    // 取外部（上层）给的 setpoint，用作基底
    setpoint_t spIn;
    memset(&spIn, 0, sizeof(spIn));
    commanderGetSetpoint(&spIn, NULL);

    // 根据状态生成覆盖命令
    bool override = false;
    setpoint_t spOut = spIn; // 复制，尽量保持 z/yaw 等通道

    if (state == OA_AVOID_FRONT) {
      // 禁止正向推进；可选微后退保持距离
      // 误差使用 "clear 阈值 - 当前距离"（单位 m）
      float eF_m = ((float)D_FRONT_CLEAR_MM - f_mm) / 1000.0f; // >0 表示太近
      float vx_hold = 0.0f;
      if (eF_m > 0.0f) {
        vx_hold = -CLAMP(K_HOLD * eF_m, 0.0f, V_HOLD_BACK_MAX);
      }
      // 如果上层还在给正向速度，强制不允许（取 min）
      float vx_in = spIn.velocity.x;
      float vx_cmd = fminf(vx_in, 0.0f); // 不许向前
      // 叠加轻微后退以保持距离
      vx_cmd = fminf(vx_cmd, vx_hold);

      // 侧向保持透传（除非前向也近+侧向更近，你也可以在此加入轻微侧向引导）
      float vy_cmd = spIn.velocity.y;

      // 统一限幅
      vx_cmd = CLAMP(vx_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);
      vy_cmd = CLAMP(vy_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);

      setVelocityBodyXY(&spOut, vx_cmd, vy_cmd);
      override = true;
    } else if (state == OA_AVOID_REAR) {      // ← 新增：禁止后退，可轻微前推
      float eB_m = ((float)D_BACK_CLEAR_MM - b_mm) / 1000.0f;  // >0 表示后方太近
      float vx_push = (eB_m > 0.0f) ?  CLAMP(K_HOLD * eB_m, 0.0f, V_HOLD_BACK_MAX) : 0.0f;
      float vx_in = spIn.velocity.x;
      float vx_cmd = fmaxf(vx_in, 0.0f);           // 禁止 -X（后退）
      vx_cmd = fmaxf(vx_cmd, vx_push);             // 叠加轻微 +X（前推）
      float vy_cmd = spIn.velocity.y;
      vx_cmd = CLAMP(vx_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);
      vy_cmd = CLAMP(vy_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);
      setVelocityBodyXY(&spOut, vx_cmd, vy_cmd);
      override = true;
    } else if (state == OA_AVOID_SIDE) {
      // 侧向保持：右近(sideSign=+1)→向左(Y+)，左近(-1)→向右(Y-)
      float d_mm = (sideSign > 0 ? r_mm : l_mm);
      float eS_m = ((float)D_SIDE_CLEAR_MM - d_mm) / 1000.0f; // >0 表示太近
      float vy_cmd = spIn.velocity.y;

      if (eS_m > 0.0f) {
        float vy_target = sideSign * CLAMP(K_SIDE * eS_m, 0.0f, V_SIDE_MAX);
        vy_cmd = slewLimit(vy_cmd_prev, vy_target, dt, V_SLEW_MAX);
        vy_cmd_prev = vy_cmd;
      } else {
        // 误差<=0，逐步回到透
        vy_cmd = slewLimit(vy_cmd_prev, spIn.velocity.y, dt, V_SLEW_MAX);
        vy_cmd_prev = vy_cmd;
      }

      // 前向透传，但若即将触发前向进入，也可在此预限
      float vx_cmd = spIn.velocity.x;
      if (f_mm < D_FRONT_ENTER_MM) {
        vx_cmd = fminf(vx_cmd, 0.0f);
      }

      // 限幅
      vx_cmd = CLAMP(vx_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);
      vy_cmd = CLAMP(vy_cmd, -V_PASSTHROUGH_MAX, V_PASSTHROUGH_MAX);

      setVelocityBodyXY(&spOut, vx_cmd, vy_cmd);
      override = true;
    } else {
      // NORMAL / CLEARING：不写 setpoint，外部任务继续
    }

    // 仅在接管时写入高优先级 setpoint；否则完全透传
    if (override) {
      commanderSetSetpoint(&spOut, CMD_PRIORITY);
    }
  }
}
