#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "led.h"

#include "commander.h"

#include "self_healing_state.h"
#include "self_healing_math.h"

static bool mission_mode = false;
static uint8_t my_hop_count = 255;
static uint32_t last_ack_timestamp = 0;
static uint32_t last_print_time = 0;
static uint8_t force_signal_loss = 0; // 0 = Segnale Ok, 1 = Interferenza finta per provare algoritmo
static uint8_t my_id = 0;
static uint32_t last_force_print_time = 0; // Timer per il log della telemetria forze

#define POSITION_UPDATE 0
#define HELP_PROXY      1

typedef struct {
    uint8_t id;
    uint8_t hop_count;
    uint8_t msg_type;
} __attribute__((packed)) SelfHealingMsg_t;

void p2pcallbackHandler(P2PPacket *p) {

    if (p->size == 0) return; //pacchetto vuoto

    SelfHealingMsg_t* incoming_msg = (SelfHealingMsg_t*)(p->data);

    uint8_t other_id = incoming_msg->id;
    
    // Ipotizzo che la Base Station abbia ID 0.
    // Se ricevo qualcosa da lei, aggiorno last ack timestamp.
    if (other_id == 0) {
        if (force_signal_loss == 0){ //Aggiorna solo se non c'è interferenza.
            last_ack_timestamp = xTaskGetTickCount();
        }
    }
    
    /*
    Filtro topologico: in una catena lineare Base-Drone1-Drone2,
   ogni nodo aggiorna la neighbor table solo con i vicini diretti
   (hop adiacenti). I nodi troppo lontani nella catena vengono ignorati
   per evitare attrazioni spurie verso nodi non raggiungibili direttamente.
   Ciò evita che in situazioni indoor i droni possano sentire direttamente la base
   al posto di prendere il segnale dal drone precedente e quindi evita calcoli di hop count 
   errati.
    */
    if (my_id != 0 && other_id < (my_id - 1)) {
        return; 
    }
    uint8_t other_hop = incoming_msg->hop_count;
    uint8_t type = incoming_msg->msg_type;
    uint8_t rssi_raw = p->rssi;
    int8_t rssi = -(int8_t)rssi_raw;

    if (type == HELP_PROXY && my_id != 0) mission_mode = true;

    uint32_t current_timestamp = xTaskGetTickCount(); 
    state_update_neighbor(other_id, rssi, other_hop, current_timestamp);

}


/*
 * Implementazione del protocollo Self-Healing su Crazyflie 2.x.
 
   Ogni drone esegue un loop periodico (ogni 250ms) che:
     1. Aggiorna la Neighbor Table con i vicini attivi nell'ultimo 1.5s
     2. Stima il proprio hop count dalla base (min hop vicino + 1)
     3. Rileva la perdita di connessione con la base (timeout ACK > 2s)
     4. Trasmette in broadcast il proprio stato (POSITION_UPDATE o HELP_PROXY)
     5. Se in mission_mode, calcola forze attrattive/repulsive e le converte
        in setpoint di velocità per il controllore di bordo.
   Ruoli:
     ID 0 = Base Station (fissa a terra, non esegue il controllore)
     ID 1 = Relay (si riposiziona per fare da ponte)
     ID 2 = Drone perso (simula la perdita di segnale via force_signal_loss)
 */
void appMain(){

    uint64_t address = configblockGetRadioAddress();
    my_id = (uint8_t)(address & 0xFF);

    //Base Station
    if (my_id == 0) {
        my_hop_count = 0; //radice della rete.
        mission_mode = false;
    }

    state_init();
    p2pRegisterCB(p2pcallbackHandler);
    DEBUG_PRINT("Self-Healing App Avviata! ID: %d\n", my_id);
    
    // Grace period di 5 secondi all'avvio: evita che il drone entri in mission_mode
    // prima che la rete si stabilizzi e i primi pacchetti siano ricevuti.
    vTaskDelay(M2T(5000));
    last_ack_timestamp = xTaskGetTickCount(); 
    mission_mode = false;

    while(1) {
        vTaskDelay(M2T(250));
        
        uint32_t current_time = xTaskGetTickCount();
        //Pulizia tabella vicini
        state_remove_dead(current_time,(uint32_t)M2T(1500));
        
        if (my_id != 0) {
            uint8_t neighbors_count_tmp = 0;
            Neighbor_t* nt = state_get_table(&neighbors_count_tmp);
            uint8_t min_hop = 254;
            for (uint8_t i = 0; i < neighbors_count_tmp; i++) {
                if (nt[i].hop_count < min_hop)
                    min_hop = nt[i].hop_count;
            }
            if (min_hop < 254) my_hop_count = min_hop + 1;
        }

        // Se è passato più di 1.5s dall'ultimo ACK della Base Station entro in mission_mode.
        if (my_id != 0){
            if ((current_time - last_ack_timestamp) > M2T(2000)) {
                mission_mode = true; 
            }
        }

        SelfHealingMsg_t HealingMSG;
        HealingMSG.id = my_id;
        HealingMSG.hop_count = my_hop_count;
        HealingMSG.msg_type = mission_mode ? HELP_PROXY : POSITION_UPDATE;
        
        P2PPacket p_reply;
        p_reply.port = 0x00;
        memcpy(p_reply.data, &HealingMSG, sizeof(SelfHealingMsg_t));
        p_reply.size = sizeof(SelfHealingMsg_t);

        radiolinkSendP2PPacketBroadcast(&p_reply);
        
        setpoint_t sp;
        memset(&sp, 0, sizeof(sp));
        
        if (my_id == 0) {
	    ledSet(LED_GREEN_R, true);
            ledSet(LED_RED_R, false);
            
            // Non chiamiamo MAI commanderSetSetpoint per la base station.
            continue;
        }
        
        //Check per vedere distanza rssi.
        if ((current_time - last_print_time) > M2T(2000)) {
	    uint8_t neighbors_count_dbg = 0;
	    Neighbor_t* nt_dbg = state_get_table(&neighbors_count_dbg);
	    // Se la tabella è vuota, stampo almeno il mio stato isolato.
            if (neighbors_count_dbg == 0) {
                DEBUG_PRINT("ID %d (Mio Hop: %d) -> Nessun vicino rilevato.\n", my_id, my_hop_count);
            }
            
            for (uint8_t i = 0; i < neighbors_count_dbg; i++) {
                float d = rssi_to_distance(nt_dbg[i].rssi);
                
                // Qui mostro sia la Neighbor Table sia l'albero di Hop
                DEBUG_PRINT("PROVA -> ID %d (Mio Hop: %d) vede ID %d (Suo Hop: %d) | DISTANZA: %.2fm (RSSI: %d)\n",
                            my_id, my_hop_count, nt_dbg[i].id, nt_dbg[i].hop_count, (double)d, (int)(int8_t)nt_dbg[i].rssi);
            }
            last_print_time = current_time;
	}
		
        sp.mode.z        = modeAbs;
	sp.position.z    = TARGET_HEIGHT;
	
	sp.mode.yaw      = modeVelocity;
	sp.attitudeRate.yaw = 0.0f;
	
	sp.mode.x = modeVelocity;
	sp.mode.y = modeVelocity;

        if(mission_mode == true && my_id != 0) {
            //Recupero tabella.
            uint8_t neighbors_count = 0;
            Neighbor_t* neighbor_table = state_get_table(&neighbors_count);
            float fx, fy, vx, vy;
            //Calcolo forze
            compute_force_vector(neighbor_table, neighbors_count, my_hop_count, my_id, &fx, &fy);
            force_to_velocity(fx, fy, &vx, &vy);

	    sp.velocity.x = vx;
	    sp.velocity.y = vy;
	    
	    if ((current_time - last_force_print_time) > M2T(2000)) {
                DEBUG_PRINT("TELEMETRIA FORZE -> ID %d | Forze calcolate: Fx=%.4f, Fy=%.4f | Output motori: Vx=%.4f, Vy=%.4f\n",
                            my_id, (double)fx, (double)fy, (double)vx, (double)vy);
                last_force_print_time = current_time;
            }
	        
	    // LED ROSSO FISSO = EMERGENZA (Mission Mode)
            ledSet(LED_RED_L, true);
            ledSet(LED_GREEN_L, false);
            ledSet(LED_BLUE_L, false);
            
        } else {
            sp.velocity.x = 0.0f;
            sp.velocity.y = 0.0f;
            
            /* 
               LED IN BASE ALL'HOP COUNT (Volo Normale).
               Feedback visivo dello stato: in volo normale i LED indicano l'hop count
               per permettere verifica visiva della corretta costruzione dell'albero di hop.
               In mission_mode il LED rosso segnala che il drone è in fase di healing.
            */
            if (my_hop_count == 0) {
                // Base: Blu
                ledSet(LED_BLUE_L, true);
                ledSet(LED_RED_L, false);
                ledSet(LED_GREEN_L, false);
            } else if (my_hop_count == 1) {
                // Drone 1: Verde
                ledSet(LED_GREEN_L, true);
                ledSet(LED_RED_L, false);
                ledSet(LED_BLUE_L, false);
            } else if (my_hop_count == 2) {
                // Drone 2: Rosso
                ledSet(LED_RED_L, true);
                ledSet(LED_GREEN_L, false);
                ledSet(LED_BLUE_L, false);
            } else {
                // Hop sconosciuto (255): Spento
                ledSet(LED_RED_L, false);
                ledSet(LED_GREEN_L, false);
                ledSet(LED_BLUE_L, false);
            }
        }
        commanderSetSetpoint(&sp, 3);
   }
}

//Per iniettare il signal loss ad un drone.
PARAM_GROUP_START(selfhealing)
PARAM_ADD(PARAM_UINT8, sig_loss, &force_signal_loss)
PARAM_GROUP_STOP(selfhealing)
