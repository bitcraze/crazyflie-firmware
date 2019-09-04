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
 * lighthouse.c: lighthouse tracking system receiver
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "lh_bootloader.h"

#include "pulse_processor.h"
#include "lighthouse.h"

#include "estimator.h"

#include "singular_value_decomposition.h"
#include "estimator_kalman.h"
#include <arm_math.h>

#ifdef LH_FLASH_DECK
#include "lh_flasher.h"
#endif

#ifndef DISABLE_LIGHTHOUSE_DRIVER
  #define DISABLE_LIGHTHOUSE_DRIVER 1
#endif

baseStationGeometry_t lighthouseBaseStationsGeometry[2]  = {
{.origin = {-0.542299, 3.152727, 1.958483, }, .mat = {{0.999975, -0.007080, -0.000000, }, {0.005645, 0.797195, 0.603696, }, {-0.004274, -0.603681, 0.797215, }, }},
{.origin = {2.563488, 3.112367, -1.062398, }, .mat = {{0.034269, -0.647552, 0.761251, }, {-0.012392, 0.761364, 0.648206, }, {-0.999336, -0.031647, 0.018067, }, }},
};

vec3d lighthouseSensorsGeometry[PULSE_PROCESSOR_N_SENSORS] = {
		{-0.0150,  0.0075, 0},
		{-0.0150, -0.0075, 0},
		{ 0.0150,  0.0075, 0},
		{ 0.0150, -0.0075, 0},
};

// Uncomment if you want to force the Crazyflie to reflash the deck at each startup
// #define FORCE_FLASH true

#define LH_FPGA_RESET DECK_GPIO_RX2

static bool isInit = false;

#if DISABLE_LIGHTHOUSE_DRIVER == 0

#ifndef FORCE_FLASH
#define FORCE_FLASH false
#endif

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bitstream, "lighthouse.bin");

static void checkVersionAndBoot();

static pulseProcessorResult_t angles[PULSE_PROCESSOR_N_SENSORS];

// Stats
static bool comSynchronized = false;

static int serialFrameCount = 0;
static int frameCount = 0;
static int cycleCount = 0;
static int positionCount = 0;

static float serialFrameRate = 0.0;
static float frameRate = 0.0;
static float cycleRate = 0.0;
static float positionRate = 0.0;

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];

static uint32_t latestStatsTimeMs = 0;

typedef union frame_u {
  struct {
    uint32_t timestamp:29;
    uint32_t sensor:3;
    uint16_t width;
    uint8_t sync;
  } __attribute__((packed));
  char data[7];
} __attribute__((packed)) frame_t;

static bool getFrame(frame_t *frame)
{
  int syncCounter = 0;
  for(int i=0; i<7; i++) {
    uart1Getchar(&frame->data[i]);
    if (frame->data[i] != 0) {
      syncCounter += 1;
    }
  }
  return (frame->sync == 0 || (syncCounter==7));
}

static void resetStats() {
  serialFrameCount = 0;
  frameCount = 0;
  cycleCount = 0;
  positionCount = 0;
}

static void calculateStats(uint32_t nowMs) {
  double time = (nowMs - latestStatsTimeMs) / 1000.0;
  serialFrameRate = serialFrameCount / time;
  frameRate = frameCount / time;
  cycleRate = cycleCount / time;
  positionRate = positionCount / time;

  resetStats();
}

static vec3d position;
static positionMeasurement_t ext_pos;
static float deltaLog;

static void estimatePosition(pulseProcessorResult_t angles[]) {
  memset(&ext_pos, 0, sizeof(ext_pos));
  int sensorsUsed = 0;
  float delta;

  // Average over all sensors with valid data
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
	  positionsIsValid[sensor] = false;
		if (angles[sensor].validCount == 4) { //single sensor has gotten x & y axis from both basestations (2*2=4)
			//it is possible to get position from single basestation, using multiple sensors
			lighthouseGeometryGetPosition(lighthouseBaseStationsGeometry, (void*)angles[sensor].correctedAngles, position, &delta); //requires two base stations to be in view

			deltaLog = delta;

			memcpy(positions[sensor], position, vec3d_size);
			positionsIsValid[sensor] = true;

			ext_pos.x -= position[2];
			ext_pos.y -= position[0];
			ext_pos.z += position[1];
			sensorsUsed++;

			positionCount++;
		}
  }

  ext_pos.x /= sensorsUsed;
  ext_pos.y /= sensorsUsed;
  ext_pos.z /= sensorsUsed;

  // Make sure we feed sane data into the estimator
  if (!isfinite(ext_pos.pos[0]) || !isfinite(ext_pos.pos[1]) || !isfinite(ext_pos.pos[2])) {
    return;
  }
  ext_pos.stdDev = 0.01;
  estimatorEnqueuePosition(&ext_pos);
}


typedef struct sensorBasestationCombo_s {
  uint8_t sensor;
  uint8_t baseStation;
} __attribute__((packed)) sensorBasestationCombo_t;

typedef struct ray_s {
  uint8_t sensor;
  uint8_t baseStation;
  vec3d origin;
  vec3d direction;
} __attribute__((packed)) ray_t;


void estimatePosition2(pulseProcessor_t *state, pulseProcessorResult_t angles[])
{
	uint32_t startT = T2M(xTaskGetTickCount());


	sensorBasestationCombo_t combos[8];

	uint8_t combo_count = 0;
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {

		for (size_t baseStation = 0; baseStation < 2; baseStation++) {



			bool hasBothAxis = true;
			for (size_t axis = 0; axis < 2; axis++) {
				//require valid horizontal and vertical sweeps

				uint32_t angleTimestamp = angles[sensor].angleTimestamps[baseStation][axis];
				if(angleTimestamp == 0){
					hasBothAxis = false;
				}else {
					uint32_t deltaTimestamp = startT - angleTimestamp;
					if(deltaTimestamp > 1000){ //if not fresh enough
						hasBothAxis = false;
					}
				}

			}

			if(hasBothAxis){
				combos[combo_count].sensor = sensor;
				combos[combo_count].baseStation = baseStation;
				combo_count++;
			}


		}

  }




  if(combo_count >= 2){
    ray_t rays[8] = {0};

  	//ref: https://stackoverflow.com/a/3389591/3553367
//		memset(&rays, 1, sizeof rays); //set all of rays to 0 (not working properly for floats)
//		memset(&rays[1], 10, sizeof(struct ray_s)); //set all of only index 1 to 10
//		memset(rays[2].origin, 10, 3*sizeof(float) ); //set only index 2's origin to 10
//		#define MEMBER_SIZE(type, member) sizeof(((type *)0)->member) //ref: https://stackoverflow.com/a/3553321/3553367
//		memset(rays[2].origin, 10, MEMBER_SIZE(ray_t, origin) ); //set only index 2's origin to 10

  	uint8_t ray_count = 0;
		for (size_t combo_index = 0; combo_index < combo_count; combo_index++) {
			uint8_t sensor = combos[combo_index].sensor;
			uint8_t baseStation = combos[combo_index].baseStation;

			static vec3d direction, origin; //TODO: very weirdly, passing rays[combo_index].direction, rays[combo_index].origin by reference causes error

//			pulseProcessorApplyCalibration(state, angles); //apply calibration only when needed
			pulseProcessorApplyCalibration2(state, angles, baseStation, sensor); //apply calibration only when needed
			calc_ray_vec(&lighthouseBaseStationsGeometry[baseStation], angles[sensor].correctedAngles[baseStation][0], angles[sensor].correctedAngles[baseStation][1], direction, origin);

			rays[combo_index].sensor = sensor;
			rays[combo_index].baseStation = baseStation;
			memcpy(rays[combo_index].origin, origin, sizeof(vec3d));
			memcpy(rays[combo_index].direction, direction, sizeof(vec3d));

			ray_count++;
		}

	  if(ray_count >= 2){

			if(rays[0].sensor != rays[1].sensor || rays[0].baseStation != rays[1].baseStation){ //must have either different basestations, or differnt sensors, or both
				vec3d D = {0}; //0 by default, likely rays fall on same sensor

				if(rays[0].sensor != rays[1].sensor){ //if rays do not fall on same sensor, find the vector between sensors
					float R[3][3];
					estimatorKalmanGetEstimatedRotationMatrix(R);
					arm_matrix_instance_f32 R_mat = {3, 3, R};

					vec3d S = {};
					arm_sub_f32(lighthouseSensorsGeometry[rays[1].sensor], lighthouseSensorsGeometry[rays[0].sensor], S, vec3d_size);
					arm_matrix_instance_f32 S_mat = {3, 1, S};

					arm_matrix_instance_f32 D_mat = {3, 1, D};

					arm_mat_mult_f32(&R_mat, &S_mat, &D_mat);

				}




				vec3d pt0;
				vec3d pt1;
//				float pt0[3];
//				float pt1[3];

				lighthouseGeometryBestFitBetweenRays(rays[0].origin, rays[1].origin, rays[0].direction, rays[1].direction, D, pt0, pt1);

				vec3d pt10;
		    arm_sub_f32(pt1, pt0, pt10, vec3d_size);

				vec3d pt10_half;
				arm_scale_f32(pt10, 0.5, pt10_half, vec3d_size);

				vec3d pt_mid;
		    arm_add_f32(pt0, pt10_half, pt_mid, vec3d_size);




	//			uint32_t endT = T2M(xTaskGetTickCount());
	//			uint32_t deltaT = endT - startT;

				return;
			}
	  }
  }


}

void fpgaTriggerReset(void)
{
  pinMode(LH_FPGA_RESET, OUTPUT);
  digitalWrite(LH_FPGA_RESET, 0);
  vTaskDelay(M2T(10));
  digitalWrite(LH_FPGA_RESET, 1);
  pinMode(LH_FPGA_RESET, INPUT);
}

static void lighthouseTask(void *param)
{
  bool synchronized = false;
  int syncCounter = 0;
  char c;
  static frame_t frame;
  static pulseProcessor_t ppState = {};

  int basestation;
  int axis;

  systemWaitStart();

  fpgaTriggerReset();

#ifdef LH_FLASH_DECK
  // Flash deck bootloader using SPI (factory and recovery flashing)
  lhflashInit();
  lhflashFlashBootloader();
#endif

  // Boot the deck firmware
  checkVersionAndBoot();

  while(1) {
    // Synchronize
    syncCounter = 0;
    while (!synchronized) {
      
      uart1Getchar(&c);
      if (c != 0) {
        syncCounter += 1;
      } else {
        syncCounter = 0;
      }
      synchronized = syncCounter == 7;
    }

    comSynchronized = true;
    DEBUG_PRINT("Synchronized!\n");

    // Receive data until being desynchronized
    synchronized = getFrame(&frame);
    while(synchronized) {
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        memset(pulseWidth, 0, sizeof(pulseWidth[0])*PULSE_PROCESSOR_N_SENSORS);
        continue;
      }

      serialFrameCount++;

      pulseWidth[frame.sensor] = frame.width;

      if (pulseProcessorProcessPulse(&ppState, frame.sensor, frame.timestamp, frame.width, angles, &basestation, &axis)) {
      	// an angle was successfully measured
        frameCount++;

        if (basestation == 1 && axis == 1) { // 4 frames per cycle: BS0-AX0, BS0-AX1, BS1-AX0, BS1-AX1
          cycleCount++;


          pulseProcessorApplyCalibration(&ppState, angles);
          estimatePosition(angles);
//					pulseProcessorApplyCalibration2(&ppState, angles, basestation, frame.sensor);
//					estimatePosition2(&ppState, angles);
//
          for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
            angles[sensor].validCount = 0;
          }
        }

//				estimatePosition2(&ppState, angles); //runs too often, will miss pulses
      }

      uint32_t nowMs = T2M(xTaskGetTickCount());
      if ((nowMs - latestStatsTimeMs) > 1000) {
        calculateStats(nowMs);
        latestStatsTimeMs = nowMs;
      }

      synchronized = getFrame(&frame);
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        continue;
      }
    }
  }
}

static void checkVersionAndBoot()
{

  uint8_t bootloaderVersion = 0;
  lhblGetVersion(&bootloaderVersion);
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Checking the bitstreams are identical
  // Also decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);
  int embeddedVersion = strtol((char*)&bitstream[2], NULL, 10);

  bool identical = true;
  for (int i=0; i<=bitstreamSize; i+=64) {
    int length = ((i+64)<bitstreamSize)?64:bitstreamSize-i;
    lhblFlashRead(LH_FW_ADDR + i, length, (uint8_t*)deckBitstream);
    if (memcmp(deckBitstream, &bitstream[i], length)) {
      DEBUG_PRINT("Fail comparing firmware\n");
      identical = false;
      break;
    }
  }

  if (identical == false || FORCE_FLASH) {
    DEBUG_PRINT("Deck has version %d and we embeed version %d\n", deckVersion, embeddedVersion);
    DEBUG_PRINT("Updating deck with embedded version!\n");

    // Erase LH deck FW
    lhblFlashEraseFirmware();

    // Flash LH deck FW
    if (lhblFlashWriteFW((uint8_t*)bitstream, bitstreamSize)) {
      DEBUG_PRINT("FW updated [OK]\n");
    } else {
      DEBUG_PRINT("FW updated [FAILED]\n");
    }
  }

  // Launch LH deck FW
  DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
  lhblBootToFW();
}

static void lighthouseInit(DeckInfo *info)
{
  if (isInit) return;

  uart1Init(230400);
  lhblInit(I2C1_DEV);
  
  xTaskCreate(lighthouseTask, "LH",
              2*configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}


static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lighthouseInit,
};


DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, rawAngle0x, &angles[0].angles[0][0])
LOG_ADD(LOG_FLOAT, rawAngle0y, &angles[0].angles[0][1])
LOG_ADD(LOG_FLOAT, rawAngle1x, &angles[0].angles[1][0])
LOG_ADD(LOG_FLOAT, rawAngle1y, &angles[0].angles[1][1])
LOG_ADD(LOG_FLOAT, angle0x, &angles[0].correctedAngles[0][0])
LOG_ADD(LOG_FLOAT, angle0y, &angles[0].correctedAngles[0][1])
LOG_ADD(LOG_FLOAT, angle1x, &angles[0].correctedAngles[1][0])
LOG_ADD(LOG_FLOAT, angle1y, &angles[0].correctedAngles[1][1])

LOG_ADD(LOG_FLOAT, angle0x_1, &angles[1].correctedAngles[0][0])
LOG_ADD(LOG_FLOAT, angle0y_1, &angles[1].correctedAngles[0][1])
LOG_ADD(LOG_FLOAT, angle1x_1, &angles[1].correctedAngles[1][0])
LOG_ADD(LOG_FLOAT, angle1y_1, &angles[1].correctedAngles[1][1])

LOG_ADD(LOG_FLOAT, angle0x_2, &angles[2].correctedAngles[0][0])
LOG_ADD(LOG_FLOAT, angle0y_2, &angles[2].correctedAngles[0][1])
LOG_ADD(LOG_FLOAT, angle1x_2, &angles[2].correctedAngles[1][0])
LOG_ADD(LOG_FLOAT, angle1y_2, &angles[2].correctedAngles[1][1])

LOG_ADD(LOG_FLOAT, angle0x_3, &angles[3].correctedAngles[0][0])
LOG_ADD(LOG_FLOAT, angle0y_3, &angles[3].correctedAngles[0][1])
LOG_ADD(LOG_FLOAT, angle1x_3, &angles[3].correctedAngles[1][0])
LOG_ADD(LOG_FLOAT, angle1y_3, &angles[3].correctedAngles[1][1])

LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])

LOG_ADD(LOG_FLOAT, delta, &deltaLog)

LOG_ADD(LOG_FLOAT, serRt, &serialFrameRate)
LOG_ADD(LOG_FLOAT, frmRt, &frameRate)
LOG_ADD(LOG_FLOAT, cycleRt, &cycleRate)
LOG_ADD(LOG_FLOAT, posRt, &positionRate)

LOG_ADD(LOG_UINT16, width0, &pulseWidth[0])
#if PULSE_PROCESSOR_N_SENSORS > 1
LOG_ADD(LOG_UINT16, width1, &pulseWidth[1])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 2
LOG_ADD(LOG_UINT16, width2, &pulseWidth[2])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 3
LOG_ADD(LOG_UINT16, width3, &pulseWidth[3])
#endif

LOG_ADD(LOG_UINT8, comSync, &comSynchronized)
LOG_GROUP_STOP(lighthouse)

#endif // DISABLE_LIGHTHOUSE_DRIVER

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bdLighthouse4, &isInit)
PARAM_GROUP_STOP(deck)
