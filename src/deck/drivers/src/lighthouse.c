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

#include "estimator_kalman.h"
#include <arm_math.h>

#ifdef LH_FLASH_DECK
#include "lh_flasher.h"
#endif

#ifndef DISABLE_LIGHTHOUSE_DRIVER
  #define DISABLE_LIGHTHOUSE_DRIVER 1
#endif

//Origins and Rotation Matrixes converted to World, because Rotation Matrix from Kalman Estimator is wrt World
baseStationGeometry_t lighthouseBaseStationsGeometry[2]  = {
{.origin = { 1.736229,  2.611738, 2.682860, }, .mat = {{-0.855681, -0.345626, 0.385167, }, {0.516858, -0.607955, 0.602701, }, {0.025856, 0.714796, 0.698855, }, }},
{.origin = {-1.372339, -2.375781, 2.739366, }, .mat = {{0.840995, 0.307949, -0.444853, }, {-0.534727, 0.598345, -0.596699, }, {0.082423, 0.739696, 0.667874, }, }},
};

// Uncomment if you want to force the Crazyflie to reflash the deck at each startup
// #define FORCE_FLASH true

static bool isInit = false;

#if DISABLE_LIGHTHOUSE_DRIVER == 0

//Sensor Positions wrt World
vec3d lighthouseSensorsGeometry[PULSE_PROCESSOR_N_SENSORS] = {
		{-0.0150,  0.0075, 0},
		{-0.0150, -0.0075, 0},
		{ 0.0150,  0.0075, 0},
		{ 0.0150, -0.0075, 0},
};

vec3d S[PULSE_PROCESSOR_N_SENSORS][PULSE_PROCESSOR_N_SENSORS];

#define LH_FPGA_RESET DECK_GPIO_RX2

static uint32_t TS_DIFF(uint32_t x, uint32_t y) {
  const uint32_t bitmask = (1 << TIMESTAMP_BITWIDTH) - 1;
  return (x - y) & bitmask;
}

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

static uint16_t serialFrameCount = 0;
static uint16_t frameCount = 0;
static uint16_t positionCount = 0;

static float serialFrameRate = 0.0;
static float frameRate = 0.0;
static float positionRate = 0.0;
//static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];

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
  uint8_t syncCounter = 0;
  for(uint8_t i=0; i<7; i++) {
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
  positionCount = 0;
}

static void calculateStats(uint32_t nowMs) {
  double time = (nowMs - latestStatsTimeMs) / 1000.0;
  serialFrameRate = serialFrameCount / time;
  frameRate = frameCount / time;
  positionRate = positionCount / time;

  resetStats();
}

static positionMeasurement_t ext_pos;
static float deltaLog;

typedef struct ray_s {
  uint8_t sensor;
  uint8_t baseStation;
} __attribute__((packed)) ray_t;


void estimatePosition(pulseProcessor_t *state, pulseProcessorResult_t angles[])
{

	#define MAX_RAYS PULSE_PROCESSOR_N_SENSORS*2  //given 2 base stations and 4 sensors, only possible to obtain in total of 8 unique rays
  ray_t rays[MAX_RAYS] = {0}; //stores all possible rays between all & and basestations

	uint8_t rays_count = 0;
	uint8_t uniqueSensorsInvolved = 0; //number of different sensors to be involved in the position calculation
	bool isBaseStationInvolved[2] = {0}; //different basestations to be involved
	{
		uint32_t startT = T2M(xTaskGetTickCount());
		for (uint8_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
			bool isSensorInvolved = false; //has the sensor in this loop been involved already
			for (uint8_t baseStation = 0; baseStation < 2; baseStation++) {

				uint8_t validAxisCount  = 0;
				for (uint8_t axis = 0; axis < 2; axis++) {
					//require valid horizontal and vertical sweeps

	//				uint32_t timestamp = angles[sensor].timestamps[baseStation][axis];
					uint32_t timestamp_i = angles[sensor].timestamps_i[baseStation][axis];

	//				if(timestamp != 0 && timestamp_i != 0) {
					if(timestamp_i != 0) {

  //				// possible need to compare timestamps from FPGA point of view, but overflows in 11 sec
	//				sweepDirection_j = X = 0
	//				sweepDirection_k = Y = 1
	//				uint32_t currentSync;
	//				if(baseStation == 0 && axis == 0){
	//					currentSync = state->currentSync0X;
	//				}else if(baseStation == 0 && axis == 1){
	//					currentSync = state->currentSync0Y;
	//				}else if(baseStation == 1 && axis == 0){
	//					currentSync = state->currentSync1X;
	//				}else if(baseStation == 1 && axis == 1){
	//					currentSync = state->currentSync1Y;
	//				}
	//				uint32_t currentSync = state->currentSync;

	//				int delta = TS_DIFF(currentSync, timestamp); //delta wrt FPGA
						int delta_i = startT - timestamp_i; //delta wrt CF

	//					if(delta < 400000){ //1 sweep/frame/axis only
	//					if(delta < 400000*2){ //2 sweeps/frames/axes, 1 cycle

//					if(delta_i < 27){//previous readings just slightly older than 1.5 cycles (more accurately 1/60*1000 * 1.5) (min steps: 0, 8/9, 16/17, 25/26)
	//					if(delta_i < 34){//previous readings just slightly older than 2 cycles (more accurately 1/60*1000 * 2)

						if(delta_i < 31){//previous readings cannot never be older than 2 cycles (1/60*1000 * 2), but capture all possible 1.5 cycles (more accurately 1/60*1000 * 1.5) (min steps: 0, 8/9, 16/17, 25/26)
							validAxisCount++;//should be 60hz per sensor
						}
					}
				}

				if(validAxisCount >= 2){

					pulseProcessorApplyCalibration2(state, angles, baseStation, sensor); //makes sure the angles for the sensor and basestation are calibrated
					//TODO: should cache the calibrations

					rays[rays_count].sensor = sensor;
					rays[rays_count].baseStation = baseStation;

					rays_count++;

					isBaseStationInvolved[baseStation] = true; //particular basestation was involved

					if(!isSensorInvolved){
						isSensorInvolved = true; //the sensor in this loop been involved already
						uniqueSensorsInvolved++; //increment the number of different sensors involved
					}

				}

			}

		}
	}

	//for every invocation, not necessarily rays are unique, but out of all the rays, one of them will surely be new (angle from an axis was updated)
	//so the combination of rays is unique as a whole (some may have been used in previous invocation)

	bool noSingleBaseStation = isBaseStationInvolved[0] && isBaseStationInvolved[1] && uniqueSensorsInvolved >= 2; //condition to ignore pairs of rays from only 1 BS (error prone)

	//  if(rays_count >= 8){ //test if all 8 rays at 1 go was possible
  if(rays_count >= 2){ //require at least two rays

    uint8_t ray_pairs_count = 0;

		for (uint8_t i = 0; i < rays_count; i++) {
			for (uint8_t j = 0; j < rays_count; j++) {

				if(rays[i].sensor != rays[j].sensor || rays[i].baseStation != rays[j].baseStation){ //must have either different basestations, or different sensors, or both


					if(noSingleBaseStation && rays[i].baseStation == rays[j].baseStation){
						//for sake of efficiency,
						//just drop single basestation ray pairs if there are 2 basestations in view, and if at least 2 sensors detected
						//otherwise, continue the inefficient but necessary computation of all possible pairs
					 continue;
					}

					static vec3d D = {0}; //0 by default, likely rays fall on same sensor

					if(rays[i].sensor != rays[j].sensor){ //if rays do not fall on same sensor, find the vector between sensors
						float R[3][3]; //Estimated rotation matrix of the Sensors
						estimatorKalmanGetEstimatedRotationMatrix(R);
						arm_matrix_instance_f32 R_mat = {3, 3, R};

//						vec3d S = {}; //the vector between the 2 sensors
//						arm_sub_f32(lighthouseSensorsGeometry[rays[j].sensor], lighthouseSensorsGeometry[rays[i].sensor], S, vec3d_size);
//						arm_matrix_instance_f32 S_mat = {3, 1, S};

						arm_matrix_instance_f32 S_mat = {3, 1, S[rays[j].sensor][rays[i].sensor]}; //use pre-computed values for speed
						arm_matrix_instance_f32 D_mat = {3, 1, D};
						arm_mat_mult_f32(&R_mat, &S_mat, &D_mat);
					}


					bool fitSuccess = false;

					static vec3d pt0;
					static vec3d pt1;

					{
						static vec3d origin_i, origin_j, direction_i, direction_j;
						calc_ray_vec(&lighthouseBaseStationsGeometry[rays[i].baseStation], angles[rays[i].sensor].correctedAngles[rays[i].baseStation][0], angles[rays[i].sensor].correctedAngles[rays[i].baseStation][1], direction_i, origin_i);
						calc_ray_vec(&lighthouseBaseStationsGeometry[rays[j].baseStation], angles[rays[j].sensor].correctedAngles[rays[j].baseStation][0], angles[rays[j].sensor].correctedAngles[rays[j].baseStation][1], direction_j, origin_j);
						//TODO: should probably cache results of calc_ray_vec

						fitSuccess = lighthouseGeometryBestFitBetweenRays(origin_i, origin_j, direction_i, direction_j, D, pt0, pt1);
					}

					if (fitSuccess){

						vec3d pt_mid;
						{
							vec3d pt10_half;
							{
								vec3d pt10;
								arm_sub_f32(pt1, pt0, pt10, vec3d_size);
								arm_scale_f32(pt10, 0.5, pt10_half, vec3d_size);
							}
							arm_add_f32(pt0, pt10_half, pt_mid, vec3d_size);
						}


						if(ray_pairs_count == 0){
							memset(&ext_pos, 0, sizeof(ext_pos)); //reset ext_pos once
						}
						ext_pos.x += pt_mid[0];
						ext_pos.y += pt_mid[1];
						ext_pos.z += pt_mid[2];
						ray_pairs_count++;


						if(ray_pairs_count >= 12){
							break; //just stop if more than 12 (mainly if using values from both basestations)
						}
					}
				}
		  }
			if(ray_pairs_count >= 12){
				break; //just stop if more than 12 (mainly if using values from both basestations)
			}
	  }


		if(ray_pairs_count > 0){
			ext_pos.x /= ray_pairs_count;
			ext_pos.y /= ray_pairs_count;
			ext_pos.z /= ray_pairs_count;

		  // Make sure we feed sane data into the estimator
		  if (!isfinite(ext_pos.pos[0]) || !isfinite(ext_pos.pos[1]) || !isfinite(ext_pos.pos[2])) {
		    return;
		  }

			positionCount++; //maxes at 60hz with 1 BS, 120 hz with 2 BS,
//			positionCount += ray_pairs_count; // maxes 720hz with 1 BS, over thousands for 2 BS (exponential increase)

		  if(noSingleBaseStation){
		  	ext_pos.stdDev = 0.01;
		  }else{
			  ext_pos.stdDev = 0.05; //since position may be from single basestation, don't trust it that much
		  }
		  estimatorEnqueuePosition(&ext_pos);

		  //			uint32_t endT = T2M(xTaskGetTickCount());
		  //			uint32_t deltaT = endT - startT; //estimate time taken for all the pairs for rays and averaging

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
  uint8_t syncCounter = 0;
  char c;
  static frame_t frame;
  static pulseProcessor_t ppState = {};

  uint8_t basestation;
  uint8_t axis;

  systemWaitStart();

  fpgaTriggerReset();

#ifdef LH_FLASH_DECK
  // Flash deck bootloader using SPI (factory and recovery flashing)
  lhflashInit();
  lhflashFlashBootloader();
#endif

  // Boot the deck firmware
  checkVersionAndBoot();


	for (uint8_t j = 0; j < PULSE_PROCESSOR_N_SENSORS; j++) {
		for (uint8_t i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
			arm_sub_f32(lighthouseSensorsGeometry[j], lighthouseSensorsGeometry[i], S[j][i], vec3d_size);
		}
	}

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
        continue;
      }

      serialFrameCount++;

      if (pulseProcessorProcessPulse(&ppState, frame.sensor, frame.timestamp, frame.width, angles, &basestation, &axis)) {
      	// an angle was successfully measured
        frameCount++;

				estimatePosition(&ppState, angles);
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

LOG_ADD(LOG_FLOAT, x, &ext_pos.x)
LOG_ADD(LOG_FLOAT, y, &ext_pos.y)
LOG_ADD(LOG_FLOAT, z, &ext_pos.z)
LOG_ADD(LOG_FLOAT, delta, &deltaLog)

LOG_ADD(LOG_FLOAT, serRt, &serialFrameRate)
LOG_ADD(LOG_FLOAT, frmRt, &frameRate)
LOG_ADD(LOG_FLOAT, posRt, &positionRate)

LOG_ADD(LOG_UINT8, comSync, &comSynchronized)
LOG_GROUP_STOP(lighthouse)

#endif // DISABLE_LIGHTHOUSE_DRIVER

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bdLighthouse4, &isInit)
PARAM_GROUP_STOP(deck)
