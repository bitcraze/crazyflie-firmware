/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * deck_spi3.h - Deck-API SPI3 communication header
 */
#ifndef SPI3_H_
#define SPI3_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SPI3_BAUDRATE_21MHZ  SPI_BaudRatePrescaler_2     // 21MHz
#define SPI3_BAUDRATE_12MHZ  SPI_BaudRatePrescaler_4     // 11.5MHz
#define SPI3_BAUDRATE_6MHZ   SPI_BaudRatePrescaler_8    // 5.25MHz
#define SPI3_BAUDRATE_3MHZ   SPI_BaudRatePrescaler_16    // 2.625MHz
#define SPI3_BAUDRATE_2MHZ   SPI_BaudRatePrescaler_32    // 1.3125MHz

/**
 * Initialize the SPI.
 */
void spi3Begin(void);
void spi3BeginTransaction(uint16_t baudRatePrescaler);
void spi3EndTransaction();

/* Send the data_tx buffer and receive into the data_rx buffer */
bool spi3Exchange(size_t length, const uint8_t *data_tx, uint8_t *data_rx);

#endif /* SPI3_H_ */
