#ifndef __SELF_HEALING_MATH_H
#define __SELF_HEALING_MATH_H

#include "self_healing_state.h"

#define DRONE_MASS 0.029f        //Massa del Crazyflie in kg.
#define DELTA_T 0.05f          // Intervallo di controllo, pari al tick del loop in secondi.
#define D_SAFE 0.3f              // Distanza minima di sicurezza anti-collisione in metri.
#define V_MAX 0.15f               // Velocita massima consentita, metri/secondi.
#define K_ATT 0.8f               // Guadagno della forza attrattiva (da tarare sperimentalmente)
#define K_REP 0.5f               // Guadagno della forza repulsiva (da tarare sperimentalmente)
#define RSSI_REF -55.0f // RSSI di riferimento a 1 metro , in dBm (usato dalla formula originale e da tarare sperimentalmente)
#define N_EXPONENT 3.0f // Esponente del modello log/distance (2=campo libero, 3 o 4 = indoor).
#define D_TARGET   0.8f   // Distanza di equilibrio della molla attrattiva in metri; deve essere > D_SAFE.
#define TARGET_HEIGHT 0.4f // Quota di volo target per i droni relay in metri.

void compute_force_vector(Neighbor_t* neighbors, uint8_t count, uint8_t my_hop, uint8_t my_id, float* fx, float* fy);

void force_to_velocity(float fx, float fy, float* vx, float* vy);

float rssi_to_distance(int8_t rssi);

#endif 
