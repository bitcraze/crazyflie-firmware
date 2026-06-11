#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#include "commander.h"

#include "self_healing_state.h"
#include "self_healing_math.h"

static bool mission_mode = false;
static uint8_t my_hop_count = 255;
static uint32_t last_ack_timestamp = 0;

static uint8_t force_signal_loss = 0; // 0 = Segnale Ok, 1 = Interferenza finta per provare algoritmo

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
            mission_mode = false; // Non siamo in mission mode perchè abbiamo ricevuto l'ack.
        }
    }
    uint8_t other_hop = incoming_msg->hop_count;
    uint8_t type = incoming_msg->msg_type;
    uint8_t rssi = p->rssi;

    if (type == HELP_PROXY) mission_mode = true;

    uint32_t current_timestamp = xTaskGetTickCount(); 
    state_update_neighbor(other_id, rssi, other_hop, current_timestamp);

  DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d (Hop %d): %s\n", rssi, other_id, other_hop, (type == HELP_PROXY) ? "AIUTO" : "AGGIORNAMENTO");
}

void appMain(){

    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id = (uint8_t)(address & 0xFF);

    //Base Station
    if (my_id == 0) {
        my_hop_count = 0; //radice della rete.
        mission_mode = false;
    }

    state_init();
    p2pRegisterCB(p2pcallbackHandler);
    DEBUG_PRINT("Self-Healing App Avviata! ID: %d\n", my_id);

    while(1) {
        vTaskDelay(M2T(50));
        
        uint32_t current_time = xTaskGetTickCount();
        //Pulizia tabella vicini
        state_remove_dead(current_time,(uint32_t)M2T(1500));

        // Se è passato più di 1.5s dall'ultimo ACK della Base Station entro in mission_mode.
        if (my_id != 0){
            if ((current_time - last_ack_timestamp) > M2T(1500)) {
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

        if(mission_mode == true) {
            //Recupero tabella.
            uint8_t neighbors_count = 0;
            Neighbor_t* neighbor_table = state_get_table(&neighbors_count);
            float fx, fy, vx, vy;
            //Calcolo forze
            compute_force_vector(neighbor_table, neighbors_count, my_hop_count, &fx, &fy);
            force_to_velocity(fx, fy, &vx, &vy);
            setpoint_t sp;
            memset(&sp, 0, sizeof(sp));
            sp.mode.x = modeVelocity;
            sp.mode.y = modeVelocity;
            sp.mode.z = modeDisable;

            sp.velocity.x = vx;
            sp.velocity.y = vy;

            commanderSetSetpoint(&sp, 3);
        }
    }
}

//Per iniettare il signal loss ad un drone.
PARAM_GROUP_START(selfhealing)
PARAM_ADD(PARAM_UINT8, sig_loss, &force_signal_loss)
PARAM_GROUP_STOP(selfhealing)