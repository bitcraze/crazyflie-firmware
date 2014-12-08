/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * nrf24l01.h: nRF24L01(-p) PRX mode low level driver
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include <stdbool.h>

#include "nRF24L01reg.h"

// Init and test of the connection to the chip
void nrfInit(void);
bool nrfTest(void);

// Interrupt routine
void nrfIsr();

/*** Defines ***/
#define RADIO_RATE_250K 0
#define RADIO_RATE_1M 1
#define RADIO_RATE_2M 2

/* Low level reg access
 * FIXME: the user should not need to access raw registers...
 */
unsigned char nrfReadReg(unsigned char address, char *buffer, int len);
unsigned char nrfRead1Reg(unsigned char address);
unsigned char nrfWriteReg(unsigned char address, char *buffer, int len);
unsigned char nrfWrite1Reg(unsigned char address, char byte);

//Interrupt access
void nrfSetInterruptCallback(void (*cb)(void));

// Low level functionality of the nrf chip
unsigned char nrfNop();
unsigned char nrfFlushRx();
unsigned char nrfFlushTx();
unsigned char nrfRxLength(unsigned int pipe);
unsigned char nrfActivate();
unsigned char nrfWriteAck(unsigned int pipe, char *buffer, int len);
unsigned char nrfReadRX(char *buffer, int len);
void nrfSetChannel(unsigned int channel);
void nrfSetDatarate(int datarate);
void nrfSetAddress(unsigned int pipe, char* address);
void nrfSetEnable(bool enable);
unsigned char nrfGetStatus();
bool nrfInterruptActive(void);



#endif
