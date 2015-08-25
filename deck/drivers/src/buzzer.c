/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * buzzer.c: Play tones or melodies
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "deck.h"

#include "param.h"
#include "pm.h"
#include "log.h"
#include "piezo.h"

typedef void (*BuzzerEffect)(uint32_t timer);

/**
 * Credit to http://tny.cz/e525c1b2 for supplying the tones
 */
#define C0 16
#define Db0 17
#define D0  18
#define Eb0 19
#define E0  20
#define F0  21
#define Gb0 23
#define G0  24
#define Ab0 25
#define A0 27
#define Bb0 29
#define B0  30
#define C1  32
#define Db1 34
#define D1  36
#define Eb1 38
#define E1  41
#define F1  43
#define Gb1 46
#define G1  49
#define Ab1 51
#define A1 55
#define Bb1 58
#define B1  61
#define C2  65
#define Db2 69
#define D2  73
#define Eb2 77
#define E2  82
#define F2  87
#define Gb2 92
#define G2  98
#define Ab2 103
#define A2 110
#define Bb2 116
#define B2  123
#define C3  130
#define Db3 138
#define D3  146
#define Eb3 155
#define E3  164
#define F3  174
#define Gb3 185
#define G3  196
#define Ab3 207
#define A3 220
#define Bb3 233
#define B3  246
#define C4  261
#define Db4 277
#define D4  293
#define Eb4 311
#define E4  329
#define F4  349
#define Gb4 369
#define G4  392
#define Ab4 415
#define A4 440
#define Bb4 466
#define B4  493
#define C5  523
#define Db5 554
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define Gb5 739
#define G5  783
#define Ab5 830
#define A5 880
#define Bb5 932
#define B5  987
#define C6  1046
#define Db6 1108
#define D6  1174
#define Eb6 1244
#define E6  1318
#define F6  1396
#define Gb6 1479
#define G6  1567
#define Ab6 1661
#define A6 1760
#define Bb6 1864
#define B6  1975
#define C7  2093
#define Db7 2217
#define D7  2349
#define Eb7 2489
#define E7  2637
#define F7  2793
#define Gb7 2959
#define G7  3135
#define Ab7 3322
#define A7 3520
#define Bb7 3729
#define B7  3951
#define C8  4186
#define Db8 4434
#define D8  4698
#define Eb8 4978
/* Duration of notes */
#define W (60)
#define H (W * 2)
#define Q (W * 4)
#define E (W * 8)
#define S (W * 16)
#define ES (W*6)


typedef struct {
  uint16_t tone;
  uint16_t duration;
} Note;

typedef struct {
  uint32_t bpm;
  uint32_t ni;
  uint32_t delay;
  Note notes[80];
} Melody;

Melody melodies[] = {
    {.bpm = 120, .ni = 0, .delay = 1, .notes = {{C4, H}, {D4, H}, {E4, H}, {F4, H}, {G4, H}, {A4, H}, {B4, H}, {0xFF, 0}}},
    {.bpm = 120, .ni = 0, .delay = 1, .notes = {{C4, S}, {D4, S}, {E4, S}, {F4, S}, {G4, S}, {A4, S}, {B4, S}, {0xFF, 0}}},
    /* Imperial march from http://tny.cz/e525c1b2A */
    {.bpm = 120, .ni = 0, .delay = 1, .notes = {{A3, Q}, {A3, Q}, {A3, Q},{F3, ES}, {C4, S},
                                                {A3, Q}, {F3, ES}, {C4, S}, {A3, H},
                                                {E4, Q}, {E4, Q}, {E4, Q}, {F4, ES}, {C4, S},
                                                {Ab3, Q}, {F3, ES}, {C4, S}, {A3, H},
                                                {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
                                                {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
                                                {C4, S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {A3, S},
                                                {C4, Q}, {A3, ES}, {C4, S}, {E4, H},
                                                {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
                                                {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
                                                {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
                                                {C4,S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {C4, S},
                                                {A3, Q}, {F3, ES}, {C4, S}, {A3, H}, {0, H},
                                                {0xFF, 0}}}

};

uint8_t melody = 2;
uint8_t nmelody = 3;
unsigned int mcounter = 0;
static bool playing_sound = false;
static void melodyplayer(uint32_t counter) {
  // Sync one melody for the loop
  Melody* m = &melodies[melody];
  if (mcounter == 0) {
    if (m->notes[m->ni].tone == 0xFF)
    {
      // Loop the melody
      m->ni = 0;
    }
    // Play current note
    piezoSetFreq(m->notes[m->ni].tone);
    piezoSetRatio(127);
    mcounter = (m->bpm * 100) / m->notes[m->ni].duration;
    playing_sound = true;
    m->ni++;
  }
  else if (mcounter == 1)
  {
      piezoSetRatio(0);
  }
  mcounter--;
}

uint8_t static_ratio = 0;
uint16_t static_freq = 4000;
static void bypass(uint32_t counter)
{
  /* Just set the params from the host */
  piezoSetRatio(static_ratio);
  piezoSetFreq(static_freq);
}

uint16_t siren_start = 2000;
uint16_t siren_freq = 2000;
uint16_t siren_stop = 4000;
int16_t siren_step = 40;
static void siren(uint32_t counter)
{
  siren_freq += siren_step;
  if (siren_freq > siren_stop)
  {
    siren_step *= -1;
    siren_freq = siren_stop;
  }
  if (siren_freq < siren_start)
  {
    siren_step *= -1;
    siren_freq = siren_start;
  }
  piezoSetRatio(127);
  piezoSetFreq(siren_freq);
}


static int pitchid;
static int rollid;
static int pitch;
static int roll;
static int tilt_freq;
static int tilt_ratio;
static void tilt(uint32_t counter)
{
  pitchid = logGetVarId("stabilizer", "pitch");
  rollid = logGetVarId("stabilizer", "roll");

  pitch = logGetInt(pitchid);
  roll = logGetInt(rollid);
  tilt_freq = 0;
  tilt_ratio = 127;

  if (abs(pitch) > 5) {
    tilt_freq = 3000 - 50 * pitch;
  }

  piezoSetRatio(tilt_ratio);
  piezoSetFreq(tilt_freq);
}


unsigned int neffect = 0;
unsigned int effect = 3;
BuzzerEffect effects[] = {bypass, siren, melodyplayer, tilt};

static xTimerHandle timer;
static uint32_t counter = 0;

static void buzzTimer(xTimerHandle timer)
{
  counter++;

  if (effects[effect] != 0)
    effects[effect](counter*10);
}

static void buzzerInit(DeckInfo *info)
{
  piezoInit();

  neffect = sizeof(effects)/sizeof(effects[0])-1;
  nmelody = sizeof(melodies)/sizeof(melodies[0])-1;

  timer = xTimerCreate( (const signed char *)"buzztimer", M2T(10),
                                     pdTRUE, NULL, buzzTimer );
  xTimerStart(timer, 100);
}

PARAM_GROUP_START(buzzer)
PARAM_ADD(PARAM_UINT8, effect, &effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT8, melody, &melody)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, nmelody, &melody)
PARAM_ADD(PARAM_UINT16, freq, &static_freq)
PARAM_ADD(PARAM_UINT8, ratio, &static_ratio)
PARAM_GROUP_STOP(buzzer)

static const DeckDriver buzzer_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcBuzzer",

  .usedGpio = DECK_USING_PA2 | DECK_USING_PA3,

  .init = buzzerInit,
};

DECK_DRIVER(buzzer_deck);
