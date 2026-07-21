#include "self_healing_math.h"
#include <math.h>

/*
 Stima della distanza dal valore RSSI usando il modello log-distance path loss.
 In ambiente indoor il fenomeno del multipath fading causa forti variazioni
 del segnale (±10 dBm), rendendo la stima inaffidabile per il controllo
 del moto. Per la dimostrazione pratica si utilizza una distanza
 fissa che corrisponde alla separazione fisica reale tra i droni nel test.
 La formula originale è mantenuta commentata per riferimento.
*/

float rssi_to_distance(int8_t rssi){
    /* Implementazione originale basata su RSSI, disabilitata per il test
    float rssi_f = (float)rssi;
    float exponent = (RSSI_REF - rssi_f) / (10.0f * N_EXPONENT);
    return powf(10.0f, exponent); 
    */

    //Distanza fissa usata per la dimostrazione pratica, corrisponde alla separazione fisica tra i droni nella configurazione di test.
    (void)rssi;
    return 1.2f;
}

/*
 Assegna la direzione di forza verso un vicino nella configurazione a linea.
 In assenza di UWB o GPS non è possibile calcolare l'angolo reale verso
 il vicino. Si usa quindi un angolo deterministico basato sull'ID:
 i nodi con ID minore sono verso la base (0 rad = asse X positivo),
 quelli con ID maggiore sono nella direzione opposta (PI rad).
 Questo è valido solo nella topologia a linea usata nel test.
*/

static float get_angle_for_neighbor(uint8_t my_id, uint8_t neighbor_id) {
    // Configurazione a linea: Base -- Drone1 -- Drone2
    if (neighbor_id < my_id) {
        return 0.0f;        // il vicino è verso la base.
    } else {
        return 3.14159f;    // il vicino è dall'altra parte.
    }
}
void compute_force_vector(Neighbor_t* neighbors, uint8_t count, uint8_t my_hop, uint8_t my_id, float* fx, float* fy){

    *fx = 0.0f;
    *fy = 0.0f; 

    for (uint8_t i = 0; i < count; i++) {
        float d = rssi_to_distance(neighbors[i].rssi);
        if (d < 0.01f) d = 0.01f;

        float force_magnitude = 0.0f;
        // Forza attrattiva verso hop precedenti e successivi. Segno positivo quindi la molla è troppo tirata e deve avvicinarsi.
        if (neighbors[i].hop_count == (my_hop - 1) || neighbors[i].hop_count == (my_hop + 1)) {
            force_magnitude += K_ATT * (d - D_TARGET);
        }

        // Forza repulsiva verso tutti i vicini troppo vicini. Segno negativo, molla troppo rilassata. Più la distanza si avvicina a zero, più la forza repulsiva esplode verso l'infinito.
        if (d < D_SAFE) {
            force_magnitude -= K_REP / (d*d);
        }

        // Proiezione della forza scalare sugli assi X e Y tramite angolo geometrico.
        float theta = get_angle_for_neighbor(my_id, neighbors[i].id);

        // Scompongo la magnitudo della forza sugli assi x e y.
        *fx += force_magnitude * cosf(theta);
        *fy += force_magnitude * sinf(theta);
    }
}

static float v_old_x = 0.0f;
static float v_old_y = 0.0f;

//Clamp sulla velocità così da non superare i valori massimi impostati.
static float clamp_velocity(float v, float max_v) {
    if (v > max_v) return max_v;
    if (v < -max_v) return -max_v;
    return v;
}


/*
 Converte il vettore di forza in velocità usando l'integrazione di Eulero.
 Applica la seconda legge di Newton (a = F/m) e integra l'accelerazione
 sulla velocità precedente. La velocità risultante è limitata a V_MAX
 per ciascun asse per ragioni di sicurezza.
 Le variabili v_old_x/y mantengono lo stato tra le chiamate successive.
*/
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
