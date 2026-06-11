#include "self_healing_math.h"
#include <math.h>

float rssi_to_distance(int8_t rssi){
    float rssi_f = (float)rssi;


    float exponent = (RSSI_REF - rssi_f) / (10.0f * N_EXPONENT);
    return powf(10.0f, exponent);
}

// Funzione helper per l'esperimento: assegno un angolo finto in base all'ID.
// Moltiplichiamo l'ID per un valore in radianti per "sparpagliare" i droni 
// in direzioni diverse (es. ID 1 = 2 rad, ID 2 = 4 rad, ecc ecc)
static float get_fake_angle_for_id(uint8_t id) {
    return (float)id * 2.0f;
}

void compute_force_vector(Neighbor_t* neighbors, uint8_t count, uint8_t my_hop, float* fx, float* fy){

    *fx = 0.0f;
    *fy = 0.0f; 

    for (uint8_t i = 0; i < count; i++) {
        float d = rssi_to_distance(neighbors[i].rssi);
        if (d < 0.01f) d = 0.01f;

        float force_magnitude = 0.0f;
        // Forza attrattiva verso hop precedenti e successivi. Segno positivo quindi la molla è troppo tirata e deve avvicinarsi.
        if (neighbors[i].hop_count == (my_hop - 1) || neighbors[i].hop_count == (my_hop + 1)) {
            force_magnitude += K_ATT * d;
        }

        // Forza repulsiva verso tutti i vicini troppo vicini. Segno negativo, molla troppo rilassata.
        // Più la distanza si avvicina a zero, più la forza repulsiva esplode verso l'infinito.
        if (d < D_SAFE) {
            force_magnitude -= K_REP / (d*d);
        }

        // Calcolo dell'angolo finto per il vicino.
        float theta = get_fake_angle_for_id(neighbors[i].id);

        // Scompongo la magnitudo della forza sugli assi x e y
        *fx += force_magnitude * cosf(theta);
        *fy += force_magnitude * sinf(theta);
    }
}

static float v_old_x = 0.0f;
static float v_old_y = 0.0f;

static float clamp_velocity(float v, float max_v) {
    if (v > max_v) return max_v;
    if (v < -max_v) return -max_v;
    return v;
}

void force_to_velocity(float fx, float fy, float* vx, float* vy){
    //Accelerazione
    float ax = fx / DRONE_MASS;
    float ay = fy / DRONE_MASS;

    //Integrazione della velocità
    float v_new_x = v_old_x + (ax * DELTA_T);
    float v_new_y = v_old_y + (ay * DELTA_T);

    //Clamp sulla velocità
    v_new_x = clamp_velocity(v_new_x, V_MAX);
    v_new_y = clamp_velocity(v_new_y, V_MAX);

    *vx = v_new_x;
    *vy = v_new_y;

    //Aggiorniamo anche gli old.
    v_old_x = v_new_x;
    v_old_y = v_new_y;
}
