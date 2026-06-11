#ifndef __SELF_HEALING_MATH_H
#define __SELF_HEALING_MATH_H

#include "self_healing_state.h"

#define DRONE_MASS 0.029f        // kg
#define DELTA_T 0.05f          // secondi
#define D_SAFE 2.0f              // metri (Distanza di sicurezza/target)
#define V_MAX 1.0f               // m/s
#define K_ATT 0.5f               // Gain attrazione (da tarare)
#define K_REP 0.8f               // Gain repulsione (da tarare)
#define RSSI_REF -55.0f // RSSI a 1 metro di distanza circa, varia da 45 fino a 60/65.
#define N_EXPONENT 2.0f // Esponente ideale per campo libero, altrimenti all'interno meglio da 3 a 4.

void compute_force_vector(Neighbor_t* neighbors, uint8_t count, uint8_t my_hop, float* fx, float* fy);

void force_to_velocity(float fx, float fy, float* vx, float* vy);

float rssi_to_distance(int8_t rssi);

#endif 