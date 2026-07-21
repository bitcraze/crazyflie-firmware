#include "self_healing_state.h"
#include <string.h>
#include <stdlib.h>

/*
 Gestione della Neighbor Table: 
 La neighbor table è una struttura dati che mantiene le informazioni
 sui droni vicini ricevute tramite broadcast P2P nell'ultimo timeout_ms millisecondi.
 La tabella ha capacità MAX_NEIGHBORS; quando è piena, il vicino meno recente
 viene sostituito (si usa una politica LRU - Least Recently Updated).
*/

static Neighbor_t neighbor_table[MAX_NEIGHBORS];
static uint8_t active_neighbors_count = 0;


// Azzera la Neighbor Table all'avvio. Da chiamare una sola volta in appMain().
void state_init(void) {
    memset(neighbor_table, 0, sizeof(neighbor_table));
    active_neighbors_count = 0;
}

// Restituisce l'indice del vicino meno recentemente aggiornato (politica LRU).
// Usata quando la tabella è piena e arriva un nuovo vicino da inserire.
uint8_t find_best_candidate_to_replace() {
    uint8_t candidate_idx = 0;
    for (uint8_t i = 1; i < active_neighbors_count; i++){
        if (neighbor_table[i].last_updated < neighbor_table[candidate_idx].last_updated) {
            candidate_idx = i;
        }
    }
    return candidate_idx;
}


// Inserisce o aggiorna un vicino nella tabella.
// Se il vicino esiste già (stesso ID) aggiorna i valori.
// Se la tabella è piena, sostituisce il vicino meno recente.
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


// Rimuove dalla tabella i vicini che non trasmettono da più di timeout_ms ms.
// Usa swap con l'ultimo elemento per evitare spostamenti in memoria.
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
