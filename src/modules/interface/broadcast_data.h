/*
 * broadcast_data.h
 *
 *  Created on: Mar 30, 2018
 *      Author: Xintong Du
 *      Email: xintong.du@mail.utoronto.ca
 */

#ifndef BROADCAST_DATA_H_
#define BROADCAST_DATA_H_

#include <stdint.h>



// ----------------------------------------------------------------------- //
//                            Data Compression                             //
// ----------------------------------------------------------------------- //

static float const POSITION_LIMIT = 8.0f; // meters
typedef int16_t posFixed16_t;
typedef struct posFixed24_t
{
  uint8_t low;
  uint8_t middle;
  uint8_t high;
} posFixed24_t;


static const uint32_t INT24_MAX = 8388607;

static inline posFixed16_t position_float_to_fix16(float x)
{
  return (INT16_MAX / POSITION_LIMIT) * x;
}

static inline float position_fix16_to_float(posFixed16_t x)
{
  return (POSITION_LIMIT / INT16_MAX) * ((float)x);
}

static inline posFixed24_t position_float_to_fix24(float x)
{
  uint32_t val = (INT24_MAX / POSITION_LIMIT) * (x + POSITION_LIMIT);
  posFixed24_t result;
  result.low = (val >> 0) & 0xFF;
  result.middle = (val >> 8) & 0xFF;
  result.high = (val >> 16) & 0xFF;
  return result;
}

static inline float position_fix24_to_float(posFixed24_t x)
{
  uint32_t val = (x.low) | (x.middle << 8) | (x.high << 16);
  return (POSITION_LIMIT / INT24_MAX) * ((float)val) - POSITION_LIMIT;
}

// ----------------------------------------------------------------------- //
//                            Crtp Packet Data struct                      //
// ----------------------------------------------------------------------- //

typedef struct data_vicon {
  struct {
    uint8_t id;
    posFixed24_t x; // m
    posFixed24_t y; // m
    posFixed24_t z; // m
    posFixed24_t yaw; // rad [CHANGE] yaw estimation
  } __attribute__((packed)) pose[2];
} __attribute__((packed)) crtp_vicon_t;

typedef struct data_setpoint {
  struct {
    uint8_t id;
    uint8_t type;
    posFixed24_t x; // m
    posFixed24_t y; // m
    posFixed24_t yaw; // rad
    posFixed24_t z; // m
   } __attribute__((packed)) pose[2];
} __attribute__((packed)) crtp_setpoint_t;


#endif /* SRC_MODULES_INTERFACE_BROADCAST_DATA_H_ */
