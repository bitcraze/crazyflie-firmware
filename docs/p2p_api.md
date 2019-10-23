---
title: Peer to Peer API
page_id: p2p_api
---

## Introduction
Currently peer to peer communication on the crazyflie is **in development**. Now an API is made available to send P2P 
messages in broadcast mode, and we are going to extend this to unicast. 

## Peer to Peer broadcast

Make sure to compile the NRF software without bluetooth support! So:

    make clean
    make BLE=0
    make cload BLE=0 

#### Sending P2P broadcast


From the STM, a packet can be sent by the following function
 
    radiolinkSendP2PPacketBroadcast()

which exists in *src/hal/interface/radiolink.h*. Make sure that the packet port is 
0x00 for now and that the packet data length does not exceed **60** entries.
#### Receiving P2P broadcast

If you want to receive packet in your function, you can register a callback with:

    p2pRegisterCB(p2pcallbackHandler);

    
with a function of the form of:

	void p2pcallbackHandler(P2PPacket *p)
	{
        ...
	
	}

