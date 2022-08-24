---
title: Token Ring-P2P API
page_id: DTR_p2p_api
---

## Introduction
An extra layer of communication is added to the Crazyflie API to allow for robust and more reliable peer to peer communication. This is done by implementing a token ring protocol on top of the peer to peer API.The Token Ring Protocol is assuming that the nodes are static and the communication is not changing. The protocol is implemented by using a token ring structure. The token ring structure is a circular linked list of nodes. The nodes are connected by a ring of links. The ring is formed by the nodes in the order they are connected. The first node is connected to the last node and the last node is connected to the first node. This protocol is used to ensure that each time only one Crazyflie broadcasts data which leads to less packet collisions and losses. It also provides a way to ensure that the transmitted data will be sent to the other copters since it receives an acknowledgement from each receiver.

Currently, peer to peer communication on the Crazyflie along with Token Ring Protocol are **in development**. An API is made available to send and receive packets to and from other Crazyflies by letting the protocol automatically handle the lower level interactions with P2P API. The protocol runs as a separate task and utilizes a separate port of the P2P API. Thus the user is given the freedom to use both the P2P API and the DTR protocol at the same time,depending on the use case and the problem to be solved.

The interface with the Token Ring Protocol is achieved by using 2 queues. One queue is used to send messages to the protocol and the other queue is used to receive messages from the protocol. In that sense ,the execution of the protocol won't block the execution of the rest of the user code and the user can send and receive messages asynchronously.


## Using the Token Ring API
Functions and structures are defined in the header files `src/modules/interface/p2pDTR/DTR_types.h` and `src/modules/interface/p2pDTR/token_ring.h`. There is also an app layer example (`/app_p2p_DTR/`) available in the example folder of the repository.

Each packet has the following structure:

``` C
typedef struct radio_packet {
	uint8_t packetSize;
	uint8_t message_type;
	uint8_t source_id;
	uint8_t target_id;
	uint8_t dataSize;
	bool allToAllFlag;
	uint8_t data[MAXIMUM_DTR_PACKET_DATA_SIZE];
} DTRpacket;
```

Where PacketSize is the size of the packet in bytes, MessageType is the type of the message, SourceID is the ID of the source, TargetID is the ID of the target, DataSize is the size of the data in bytes, AllToAllFlag is a flag indicating if the message is to be sent to all the other Crazyflies or not, and Data is the data of the message.The maximum size of the data is 53 since the data for the P2P is 60 and the protocol occupies 7 of them for operation.

## Setting up the Token Ring Protocol
Since the nodes od the token ring are static, the user has to define the topology of the network. The topology is defined by the number of nodes and their ids in the order they are connected. The topology is defined like following:

``` C
#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default
static DTRtopology topology = NETWORK_TOPOLOGY;

...

void main() {
  ...
  // Start the token ring protocol
  dtrEnableProtocol(topology);
  ...
}
```

In the example above, the topology is defined as having 4 nodes and their ids are 1, 0, 2, 3.

## Feeding incoming packets to the protocol
In order for the protocol to work, the user must feed the protocol with incoming packets. This is done by calling the function `dtrP2PIncomingHandler` which in the interface for receiving P2P packets. As mentioned above the protocol uses the port 15 of the P2P API, so it automatically checks if the packet is coming from the port 15 and if it is, it handles it. Otherwise it is discarded.The function returns true if the packet was handled and false otherwise.
 

Example usage :
``` C
void p2pcallbackHandler(P2PPacket *p){
	// If the packet is a DTR service packet, then the handler will handle it.
   if (!dtrP2PIncomingHandler(p)){
		// If packet was not handled from DTR , then it is a normal packet 
		// that user has to handle. 
		
		// Write your own code below ...
	}
}

...


void main(){
  ...
  // Register the callback function to handle incoming packets.
  p2pRegisterCallback(p2pcallbackHandler);
  ...
}

```


## Data Broadcast

To send data through the protocol, the user must call the function `dtrSendPacket`. 
  ``` C
  bool dtrSendPacket(DTRpacket* packet)
  ```
This function takes the packet to be sent as a parameter. The packet must be filled wit the data the user wants to send and by defining the size of them. The function returns true if the packet was sent successfully to the DTR (**not to the receiver copter**) and false  otherwise.Keep in mind that the packet is sent asynchronously and the user can continue to use the P2P API while the packet is being sent.It is not necessary to fill the `.source_id`, `message_type`, `packet_size` fields of the packet since they are automatically filled by the API function. After the execution of the function, the new packet is inserted in the queue of the protocol responsible for all the packets to be sent. 
  


  Example Usage:
  ``` C
  // Initialize the packet
  DTRpacket packet;

  // Fill the packet with the data
  packet.dataSize = 3;
  packet.data[0] = 0x01;
  packet.data[1] = 0x02;
  packet.data[2] = 0x03;
  packet.allToAllFlag = 1;
  dtrSendPacket(&packet);

  ```


## Receive Data

The user must call the function `dtrGetPacket` to receive a packet from the DTR. 
``` C
bool dtrGetPacket(DTRpacket* packet, uint32_t timeout);
```

The function blocks for the specified time (in ticks) until a packet is received. If the user wants to block indefinitely, the timeout parameter must be set to `portMAX_DELAY`. The function returns true if a packet was received and false otherwise. If a packet is received, the packet is filled with the data received. In case it was received, the packet is filled with the data received and the corresponding packet is released from the queue responsible for the reception of the DTR packets.
Example Usage:
``` C
// Initialize the packet
DTRpacket packet;

// Receive the packet
while(1){
  dtrGetPacket(&packet, portMAX_DELAY);
  
  if(packet.dataSize > 0){
    // Do something with the packet
    for (int i = 0; i < packet.dataSize; i++){
      printf("%d ", packet.data[i]);
    }
    
    printf("\n");
  }
}
```
