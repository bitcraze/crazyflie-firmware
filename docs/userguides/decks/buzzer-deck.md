---
title: Buzzer Deck
page_id: buzzer-deck
---

## Changing the sounds 



Changing the sounds requires modifications to the firmware. The code for the sounds is located in [sound_cf.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/sound_cf2.c). 

A sequences is defined in the .notes member of the Melody struct. The notes are defined as tuples of a note (pitch) and duration. A sequence must be ended with the end marker.

The effects are defined in the effects array. Each entry takes a function to call and related parameters.
To add a new melody, use the melodyplayer function for the .call member and assign your Melody struct to the .melody member.

## Parameters 



|Name|Type|Access|Value (default)|Description|
|---|----|---|---|---
|sound.effect|uint8_t|RW|0|Sound effect (0=Off, 1=Factory test, 2=USB connected, 3=USB disconnected, 4=Charging done, 5=Low battery, 6=Startup, 7=Calibrated, 8=Range slow, 9=Range fast, 10=Star Wars Imperial March, 11=Bypass (change sound with sound.freg), 12=Siren, 13=Tilt (tilt the Crazyflie to play the sound))|
|sound.freq|uint16_t|RW|4000| |
|sound.neffect|uint32_t|RO|13|Number of available sound effects|
|sound.ratio|uint8_t|RW|0| |
