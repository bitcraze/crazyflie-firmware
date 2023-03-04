#ifndef MSG_H
#define MSG_H

#define MSG_SIZE 53
#define GAP8_ENABLED false

#include "stdio.h"

#if GAP8_ENABLED
  #include "pmsis.h"
#endif

typedef struct _Pixel
{
  uint16_t x; // uint16 (2 bytes)
  uint16_t y; // uint16 (2 bytes)
} __attribute__((packed)) Pixel;

typedef struct _TagPacket
{
  uint8_t id; // uint8 (1 byte) = 1
  Pixel corners[4]; // Pixel (4 bytes) * 4 = 16
  float homography[3][3]; // float (4 bytes) * 3 * 3 = 36
} __attribute__((packed)) TagPacket;

#if GAP8_ENABLED
  // https://stackoverflow.com/a/6002598
  static void reserve_space(uint8_t *data, size_t bytes,
    uint8_t *size, uint8_t *next) 
  {
    if((*next + bytes) > *size) {
      // double size to enforce O(lg N) reallocs
      // data = realloc(data, *size * 2);
      // *size *= 2;
    }
  }
  
  static void serialize_uint8(uint8_t x, uint8_t *data, 
    uint8_t *size, uint8_t *next) 
  {
    reserve_space(data, sizeof(uint8_t), size, next);

    memcpy(data + *next, &x, sizeof(uint8_t));
    *next += sizeof(uint8_t);
  }

  static void serialize_uint16(uint16_t x, uint8_t *data, 
    uint8_t *size, uint8_t *next) 
  {
    reserve_space(data, sizeof(uint16_t), size, next);

    memcpy(data + *next, &x, sizeof(uint16_t));
    *next += sizeof(uint16_t);
  }

  static void serialize_float(float x, uint8_t *data, 
    uint8_t *size, uint8_t *next) 
  {
    reserve_space(data, sizeof(float), size, next);

    memcpy(data + *next, &x, sizeof(float));
    *next += sizeof(float);
  }

  static void serialization(TagPacket tp, CPXPacket_t *pkt) 
  { 
    uint8_t size = MSG_SIZE;
    uint8_t next = 0;

    pkt->dataLength = (uint16_t)size;

    serialize_uint8(tp.id, pkt->data, &size, &next);
    for (size_t i = 0; i < sizeof(tp.corners)/sizeof(Pixel); ++i)
    {
      serialize_uint16(tp.corners[i].x, pkt->data, &size, &next);
      serialize_uint16(tp.corners[i].y, pkt->data, &size, &next);
    }
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
          serialize_float(tp.homography[i][j], pkt->data, &size, &next);
  }
#endif

static void deserialize_uint8(uint8_t *x, uint8_t *data, 
  uint8_t *next) 
{
  memcpy(x, data + *next, sizeof(uint8_t));
  *next += sizeof(uint8_t);
}

static void deserialize_uint16(uint16_t *x, uint8_t *data, 
  uint8_t *next) 
{
  memcpy(x, data + *next, sizeof(uint16_t));
  *next += sizeof(uint16_t);
}

static void deserialize_float(float *x, uint8_t *data, 
  uint8_t *next) 
{

  memcpy(x, data + *next, sizeof(float));
  *next += sizeof(float);
}

static void deserialization(TagPacket *tp, CPXPacket_t pkt) 
{
  uint8_t next = 0;

  deserialize_uint8(&(tp->id), pkt.data, &next);
  for (size_t i = 0; i < sizeof(tp->corners)/sizeof(Pixel); ++i)
  {
    uint16_t x = 0, y = 0;
    deserialize_uint16(&x, pkt.data, &next);
    deserialize_uint16(&y, pkt.data, &next);
    tp->corners[i].x = x;
    tp->corners[i].y = y;
  }
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      float x = 0.0f;
      deserialize_float(&x, pkt.data, &next);
      tp->homography[i][j] = x;
    }
  }
}

#endif