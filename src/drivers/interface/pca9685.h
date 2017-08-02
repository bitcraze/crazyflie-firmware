/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2017 BitCraze AB
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
 * pca9685.h: 12-bit, 16-channel PWM servo (, LED, ESC, ...) driver
 */


// start the device. All channels will be initialized to 0% duty.
// returns true if successful.
bool pca9685init(int addr, float pwmFreq);

// set duty cycle of several channels at once.
// duties should lie in [0, 1] inclusive.
// invalid duties clipped silently.
// returns true if successful.
bool pca9685setDuties(
  int addr, int chanBegin, int nChan, float const *duties);

// set the (positive) pulse duration of several channels at once.
// durations should lie in [0, 4096],
// where 0 represents DC ground and 4096 represents DC Vcc.
// the actual duration value depends on the PWM frequency.
// invalid durations clipped silently.
// returns true if successful.
bool pca9685setDurations(
  int addr, int chanBegin, int nChan, uint16_t const *durations);


//
// ------------------------ Asynchronous Interface ------------------------//
//
// Large I2C writes take a long time because we wait for ACKs from the slave.
// This is a problem if, e.g., we want to drive ESCs with this device,
// and we don't want to block the stabilizer loop waiting for I2C writes.
// To deal with this problem, we implement an asynchronous interface
// using a separate task and a one-element queue.
//
// Current implementation will drop messages if async writes occur faster
// than they can be sent. This could be relaxed if needed, 
// but it's the right behavior for driving ESCs.


bool pca9685startAsyncTask();

// see sync version for description.
// returns true if the message was enqueued, false if queue is already full.
bool pca9685setDutiesAsync(
  int addr, int chanBegin, int nChan, float const *duties);

// see sync version for description.
// returns true if the message was enqueued, false if queue is already full.
bool pca9685setDurationsAsync(
  int addr, int chanBegin, int nChan, uint16_t const *durations);
