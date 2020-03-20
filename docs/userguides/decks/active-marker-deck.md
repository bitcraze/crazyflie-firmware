---
title: Active marker deck
page_id: active-marker-deck
---

The Active Marker deck is mainly designed for [Qualisys mocap systems](https://www.qualisys.com/) and supports Qualisys Active markers, but it can also be used with other systems in a simplified mode. The deck has 4 arms with one IR LED on the tip of each arm and a light sensor in the center of the deck.

The deck is configured using the [parameter sub system](/userguides/logparam/), for details on which parameter to use, see below.

## Modes

The deck mode is set with the ```activeMarker.mode``` parameter.

| Mode      | value       | coment                       |
| --------- | ----------- | ---------------------------- |
| OFF       | 0           | Always off                   |
| ON        | 1           | Always on                    |
| MODULATED | 2           | Switching on/off             |
| QUALISYS  | 3 (default) | Qualisys Active Marker mode |

### Modulated mode

The LEDs are switched on and off at around 42 kHz (24 micro seconds cycle)

### Qualisys mode

In this mode the LEDs act as Actvive markers with IDs that are identified by the Qualisys system and used for better 6-dof identification and tracking. The markers are assigned default IDs at startup, but can easily be changed through parameters. The Qualisys sytems and the deck currently supports IDs in the range [0 - 170]

| Marker position | parameter                | deafult value | type              |
| --------------- | ------------------------ | ------------- | ----------------- |
| Front           | ```activeMarker.front``` |  1            | ```PARAM_UINT8``` |
| Right           | ```activeMarker.right``` |  2            | ```PARAM_UINT8``` |
| Back            | ```activeMarker.back```  |  3            | ```PARAM_UINT8``` |
| Left            | ```activeMarker.left```  |  4            | ```PARAM_UINT8``` |
