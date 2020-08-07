---
title: Active marker deck
page_id: active-marker-deck
---

The Active Marker deck is mainly designed for [Qualisys mocap systems](https://www.qualisys.com/) and supports Qualisys Active markers, but it can also be used with other systems in a simplified mode. The deck has 4 arms with one IR LED on the tip of each arm and a light sensor in the center of the deck.

The deck is configured using the [parameter sub system](/docs/userguides/logparam.md), for details on which parameter to use, see below.

## Modes

The deck mode is set with the ```activeMarker.mode``` parameter.

| Mode      | value       | coment                       |
| --------- | ----------- | ---------------------------- |
| OFF       | 0           | Always off                   |
| ON        | 1           | Always on                    |
| MODULATED | 2           | Switching                    |
| QUALISYS  | 3 (default) | Qualisys Active Marker mode  |

### Off mode

All marker LEDs are turned off

### On mode

The marker LEDs are turned on. The brightness of each LED is controlled by the [marker parameters](#marker-parameters) below, in the range 0 - 255.

### Modulated mode

The LEDs are switched on and off at around 42 kHz (24 micro seconds cycle). The brightness of the LEDs during the "on" part of the cycle is controlled by the [marker parameters](#marker-parameters) below, in the range 0 - 255.

### Qualisys mode

In this mode the LEDs act as Actvive markers with IDs that are identified by the Qualisys system and used for better 6-dof identification and tracking. The IDs are controlled by the [marker parameters](#marker-parameters) below. The Qualisys sytems and the deck currently supports IDs in the range [0 - 170]

## Marker parameters

Each marker is associated with a parameters that is used to set brightness or id.

| Marker position | parameter                | deafult value | type              |
| --------------- | ------------------------ | ------------- | ----------------- |
| Front           | ```activeMarker.front``` |  1            | ```PARAM_UINT8``` |
| Right           | ```activeMarker.right``` |  2            | ```PARAM_UINT8``` |
| Back            | ```activeMarker.back```  |  3            | ```PARAM_UINT8``` |
| Left            | ```activeMarker.left```  |  4            | ```PARAM_UINT8``` |
