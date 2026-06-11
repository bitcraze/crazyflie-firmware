#include "self_healing_state.h"
#include <string.h>
#include <stdlib.h>

static Neighbor_t neighbor_table[MAX_NEIGHBORS];
static uint8_t active_neighbors_count = 0;

void state_init(void) {
    memset(neighbor_table, 0, sizeof(neighbor_table));
    active_neighbors_count = 0;
}

uint8_t find_best_candidate_to_replace() {
    uint8_t candidate_idx = 0;
    for (uint8_t i = 1; i < active_neighbors_count; i++){
        if (neighbor_table[i].last_updated < neighbor_table[candidate_idx].last_updated) {
            candidate_idx = i;
        }
    }
    return candidate_idx;
}

void state_update_neighbor(uint8_t id, int8_t rssi, uint8_t hop_count ,uint32_t current_timestamp){
    uint8_t write_idx = 255;
    for (uint8_t i = 0; i < active_neighbors_count; i++) {
        if (neighbor_table[i].id == id) {
            neighbor_table[i].rssi = rssi;
            neighbor_table[i].last_updated = current_timestamp;
            neighbor_table[i].hop_count = hop_count;

            return;
        }
    }

    if (active_neighbors_count < MAX_NEIGHBORS) {
        write_idx = active_neighbors_count;
        active_neighbors_count++;
    } else {
        write_idx = find_best_candidate_to_replace();
    }
    
    neighbor_table[write_idx].id = id;
    neighbor_table[write_idx].rssi = rssi;
    neighbor_table[write_idx].last_updated = current_timestamp;
    neighbor_table[write_idx].hop_count = hop_count;
}

void state_remove_dead(uint32_t current_timestamp, uint32_t timeout_ms) {
    uint8_t i = 0;
    while(i < active_neighbors_count){
        if(current_timestamp - neighbor_table[i].last_updated > timeout_ms){
            neighbor_table[i] = neighbor_table[active_neighbors_count - 1];
            active_neighbors_count -= 1;
        } else {
            i++;
        }
    }
}

Neighbor_t* state_get_table(uint8_t* count){
    *count = active_neighbors_count;
    return neighbor_table;
}