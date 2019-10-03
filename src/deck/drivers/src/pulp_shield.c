/*----------------------------------------------------------------------------*
 * Copyright (C) 2018-2019 ETH Zurich, Switzerland                            *
 * All rights reserved.                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * See LICENSE.apache.md in the top directory for details.                    *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 *                                                                            *
 * File:    pulp_shield.c                                                     *
 * Author:  Daniele Palossi <dpalossi@iis.ee.ethz.ch>                         *
 * Date:    04.09.2019                                                        *
 *----------------------------------------------------------------------------*/


#include "pulp_shield.h"

static float alpha_velocity_ 		= ALPHA_VEL;
static float alpha_yaw_ 			= ALPHA_YAW;
static float max_forward_index_ 	= MAX_FORWARD_INDEX;
static float critical_prob_coll_ 	= CRITICAL_PROB_COLL;
static float yaw_scaling			= YAW_SCALING;

static float steering_angle_;
static float probability_of_collision_;
static float desired_forward_velocity_;
static float desired_angular_velocity_;

static setpoint_t setpoint_ = {
				.position.x 		= 0.0f,
				.position.y 		= 0.0f,
				.position.z 		= 0.0f,
		
				.velocity.x 		= 0.0f,
				.velocity.y 		= 0.0f,
				.velocity.z 		= 0.0f,
		
				.acceleration.x 	= 0.0f,
				.acceleration.y 	= 0.0f,
				.acceleration.z 	= 0.0f,
		
				.attitude.pitch 	= 0.0f,
				.attitude.roll 		= 0.0f,
				.attitude.yaw 		= 0.0f,
		
				.attitudeRate.pitch = 0.0f,
				.attitudeRate.roll 	= 0.0f,
				.attitudeRate.yaw 	= 0.0f,
		
				.mode.x				= modeDisable,
				.mode.y				= modeDisable,
				.mode.z				= modeDisable,
				.mode.pitch			= modeDisable,
				.mode.roll			= modeDisable,
				.mode.yaw			= modeDisable,
				.mode.quat			= modeDisable,
		
				.velocity_body 		= true,
};

static uint32_t step, lastUpdate, sleep;
static bool isInit					= false;
static bool isEnabled 				= false;
#ifdef COLLISION_TEST
static bool stop 					= false;
#endif


static struct {
	float32_t samples[MA_HISTORY];
	size_t count;
} steeringAverages;

static struct {
	float32_t samples[MA_HISTORY];
	size_t count;
} velocityAverages;


setpoint_t PULPShieldGetSetpoint() {
	return setpoint_;
}


static void PULPShieldTask(void *param) {

	systemWaitStart();

	while(1) {

	    uint32_t currentTime = xTaskGetTickCount();
		/**** INITIAL TIMER ****/
	    if(currentTime-sleep<SLEEP_THRESHOLD) {
	    	lastUpdate = xTaskGetTickCount();
	        // reset EKF
	        setResetEstimation(true);
	    	vTaskDelay(200);
	    }
	    else {
			if(isEnabled) {
				/**** TAKE-OFF ****/
				if(step <= PULP_STEPS_TAKEOFF+PULP_STEPS_HOVERING) {
					if((currentTime-lastUpdate) > PULP_TAKEOFF_RATE) {
						DroNetTakeOff();
						lastUpdate = xTaskGetTickCount();
						step++;
					}
				}
				/**** AUTONOMOUS NAVIGATION ****/
				else {
					if((currentTime-lastUpdate) > PULP_NAVIGATE_RATE) {
						DroNetGetSetpoint();
						lastUpdate = xTaskGetTickCount();
						step++;
					}
				}
			} else {
				vTaskDelay(200);
			}
	    }
	} // close while
}


static void PULPShieldInit() {

	if(isInit) return;

	xTaskCreate(PULPShieldTask,				// pointer to the task entry function
				PULP_SHIELD_TASK_NAME,		// name for the task
				PULP_SHIELD_TASK_STACKSIZE,	// number of words for task's stack
				NULL,						// the task's parameter
				PULP_SHIELD_TASK_PRI,		// task priority
				NULL);						// pass a handle

	GPIOInit();
	GPIOOff();

    isInit = true;

	DEBUG_PRINT("PULP-Shield Initialized\n");
}


static bool PULPShieldTest() {

	DEBUG_PRINT("First PULP-Shield test passed!\n");
	// GPIO/SPI test should go here
	return true;
}


void PULPShieldOn() {

	steering_angle_ 			= 0.0f;
	probability_of_collision_	= 0.0f;
	desired_forward_velocity_	= 0.0f; //max_forward_index_;
	desired_angular_velocity_	= 0.0f;
	lastUpdate 					= xTaskGetTickCount();

	// Set GPIO on (power on the shield)
	GPIOOn();
	// Enable SPI
	spiBeginSlave();

	// Reset task's step counter
	step = 1;

	// Reset internal state
	DroNetResetSetpoint();
	for(int i=0; i<MA_HISTORY; i++) {
		steeringAverages.samples[i] = 0.0f;
		velocityAverages.samples[i] = 0.0f;
	}
	steeringAverages.count = 0;
	velocityAverages.count = 0;

    sleep = xTaskGetTickCount();

	// Enable must be set last
	isEnabled = true;
}


void PULPShieldOff() {

	// Enable must be set first
	isEnabled = false;

	// Set GPIO low
	GPIOOff();

	// Reset internal state
	DroNetResetSetpoint();

	// Reset task's step counter
	step = 1;
}


void GPIOInit() {

	//Enable clock for GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//Initialize struct
	GPIO_InitTypeDef GPIO_InitStruct;
	//Pin 12
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_12;
	//Mode output
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_OUT;
	//Output type push-pull
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;
	//Without pull resistors
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	//50MHz pin speed
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	//Initialize pins on GPIOG port
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}


void GPIOOn() {

	//Set PC12 pin HIGH
	GPIO_SetBits(GPIOC, GPIO_Pin_12);
}


void GPIOOff(){

	//Set PC12 pin LOW
	GPIO_ResetBits(GPIOC, GPIO_Pin_12);
}


void getDronetOutputs() {

	uint8_t data_rx[4] = {0x00, 0x00, 0x00, 0x00};
	uint8_t data_tx[4] = {0x00, 0x00, 0x00, 0x00};

	spiBeginTransactionSlave(SPI_BAUDRATE_2MHZ);
    spiExchange(4, data_tx, data_rx);
    spiEndTransaction();

	steering_angle_				= fixed2float((int16_t*)&data_rx[0]);
	float x						= fixed2float((int16_t*)&data_rx[2]);
	probability_of_collision_ 	= 1.0f/(1.0f + expf(-x));

	// Output modulation
	if(steering_angle_ < -1.0f) steering_angle_	= -1.0f;
	if(steering_angle_ > 1.0f) steering_angle_ 	= 1.0f;
	if(probability_of_collision_ < 0.1f) probability_of_collision_ 	= 0.0f;

#ifdef COLLISION_TEST
	steering_angle_ = 0.0f;
#endif

}


void DroNetGetSetpoint() {

    // update the output from DroNet
	getDronetOutputs();
	float desired_forward_velocity_m;

	/************************ FORWARD VELOCITY ************************/

	desired_forward_velocity_m = (1.0f - probability_of_collision_) * max_forward_index_;

	if(desired_forward_velocity_m < 0.0f) {
		DEBUG_PRINT("Detected negative forward velocity! Drone will now stop!\n");
		desired_forward_velocity_m  = 0.0f;
	}

	// Low pass filter the velocity
	desired_forward_velocity_ = (1.0f - alpha_velocity_) * desired_forward_velocity_
			+ alpha_velocity_ * desired_forward_velocity_m;

	// Stop if velocity is prob of collision is too high
	if(desired_forward_velocity_ < ((1.0f - critical_prob_coll_) * max_forward_index_)) {
		desired_forward_velocity_ = 0.0f;
	}

	/******************************************************************/


	/************************ ANGULAR VELOCITY ************************/

	// Low pass filter the angular_velocity
	desired_angular_velocity_ = (1.0f - alpha_yaw_) * desired_angular_velocity_
			+ alpha_yaw_ * steering_angle_;

	/******************************************************************/


	/*********************** STATE UPDATE RATE ************************/

	setpoint_.mode.x			= modeVelocity;
	setpoint_.mode.y			= modeVelocity;
	setpoint_.mode.z			= modeAbs;
	setpoint_.mode.yaw			= modeVelocity;
	setpoint_.position.z 		= PULP_TARGET_H;
	setpoint_.velocity.x		= desired_forward_velocity_;
	setpoint_.velocity.y 		= 0.0f;
	setpoint_.attitudeRate.yaw	= desired_angular_velocity_*yaw_scaling;

	/******************************************************************/
}


void DroNetResetSetpoint() {

	setpoint_.position.x 		= 0.0f;
	setpoint_.position.y 		= 0.0f;
	setpoint_.position.z 		= 0.0f;

	setpoint_.velocity.x		= 0.0f;
	setpoint_.velocity.y 		= 0.0f;
	setpoint_.velocity.z 		= 0.0f;

	setpoint_.acceleration.x 	= 0.0f;
	setpoint_.acceleration.y 	= 0.0f;
	setpoint_.acceleration.z 	= 0.0f;

	setpoint_.attitude.pitch	= 0.0f;
	setpoint_.attitude.roll		= 0.0f;
	setpoint_.attitude.yaw		= 0.0f;

	setpoint_.attitudeRate.pitch= 0.0f;
	setpoint_.attitudeRate.roll	= 0.0f;
	setpoint_.attitudeRate.yaw	= 0.0f;

	setpoint_.mode.x			= modeDisable;
	setpoint_.mode.y			= modeDisable;
	setpoint_.mode.z			= modeDisable;
	setpoint_.mode.roll			= modeDisable;
	setpoint_.mode.pitch		= modeDisable;
	setpoint_.mode.yaw			= modeDisable;
	setpoint_.mode.quat			= modeDisable;

	setpoint_.velocity_body 	= true;
}


void DroNetTakeOff() {

	setpoint_.position.x 		= 0.0f;
	setpoint_.position.y 		= 0.0f;
	if(step <= PULP_STEPS_TAKEOFF)
		setpoint_.position.z 	= (PULP_TARGET_H/PULP_STEPS_TAKEOFF*1.0f)*(step*1.0f);
	else
		setpoint_.position.z 	= PULP_TARGET_H;
	setpoint_.velocity.x		= 0.0f;
	setpoint_.velocity.y 		= 0.0f;
	setpoint_.velocity.z 		= 0.0f;

	setpoint_.acceleration.x 	= 0.0f;
	setpoint_.acceleration.y 	= 0.0f;
	setpoint_.acceleration.z 	= 0.0f;

	setpoint_.attitude.pitch	= 0.0f;
	setpoint_.attitude.roll		= 0.0f;
	setpoint_.attitude.yaw		= 0.0f;

	setpoint_.attitudeRate.pitch= 0.0f;
	setpoint_.attitudeRate.roll	= 0.0f;
	setpoint_.attitudeRate.yaw	= 0.0f;

	setpoint_.mode.x			= modeVelocity;
	setpoint_.mode.y			= modeVelocity;
	setpoint_.mode.z			= modeAbs;
	setpoint_.mode.roll			= modeDisable;
	setpoint_.mode.pitch		= modeDisable;
	setpoint_.mode.yaw			= modeVelocity;
	setpoint_.mode.quat			= modeDisable;

	setpoint_.velocity_body 	= true;
}


float fixed2float(int16_t *x) {
	if((~x[0]) < x[0])
		return -FIXED_BITVALUE * ((~x[0])+1);
	else
		return FIXED_BITVALUE * x[0];
}


uint32_t getLastPULPUpdate() {
	return lastUpdate;
}


static const DeckDriver PULPShieldDriver = {
		.name = "PULPShield",
		.init = PULPShieldInit,
		.test = PULPShieldTest,
};


DECK_DRIVER(PULPShieldDriver);

PARAM_GROUP_START(dronet)
PARAM_ADD(PARAM_FLOAT, alphaVel, &alpha_velocity_)
PARAM_ADD(PARAM_FLOAT, alphaYaw, &alpha_yaw_)
PARAM_ADD(PARAM_FLOAT, maxVel, &max_forward_index_)
PARAM_ADD(PARAM_FLOAT, criticalProb, &critical_prob_coll_)
PARAM_ADD(PARAM_FLOAT, yawScaling, &yaw_scaling)
PARAM_GROUP_STOP(dronet)

LOG_GROUP_START(DroNet)
LOG_ADD(LOG_FLOAT, Steering, &steering_angle_)
LOG_ADD(LOG_FLOAT, Collision, &probability_of_collision_)
LOG_GROUP_STOP(DroNet)