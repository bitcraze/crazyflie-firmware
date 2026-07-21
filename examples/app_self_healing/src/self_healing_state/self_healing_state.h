#ifndef __SELF_HEALING_STATE_H
#define __SELF_HEALING_STATE_H

#include <stdint.h>
#include <stdbool.h>

// Numero massimo di droni vicini tracciati simultaneamente nella Neighbor Table.
#define MAX_NEIGHBORS 10 

typedef struct {
    uint8_t id;             // Identificativo univoco del drone vicino.        
    int8_t rssi;            // Potenza del segnale ricevuto in dBm, valore negativo.
    uint32_t last_updated;  // Timestamp FreeRTOS dell'ultimo pacchetto ricevuto (tick).
    uint8_t hop_count;      // Distanza in salti dalla base station dichiarata dal vicino.
} Neighbor_t;

void state_init(void);

void state_update_neighbor(uint8_t id, int8_t rssi, uint8_t hop_count ,uint32_t current_timestamp);

void state_remove_dead(uint32_t current_timestamp, uint32_t timeout_ms);

Neighbor_t* state_get_table(uint8_t* count);

#endif
