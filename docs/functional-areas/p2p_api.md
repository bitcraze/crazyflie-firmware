---
title: Peer to Peer API
page_id: p2p_api
---

## Introduction
Currently peer to peer communication on the Crazyflie is **in development**. Now an API is made available to send P2P
messages in broadcast mode, and we are going to extend this to unicast.

P2P packets are sent and received on the same channel as the currently configured CRTP radio link. P2P packets are sent and received independently to regular CRTP packets. In order to allow for multiple applications to use P2P communication at the same time a port number has been added to each packet, the intend being that independent service will use different P2P port. For the time being this port is not used by the API and keeping it to 0 unless otherwise needed is advised.

Furthermore, P2P packets are only sent and received from the 2.4GHz internal Crazyflie radio independently of where the CRTP link is connect (ex. the CRTP link can be connected over USB).

The maximum data payload contained in a P2P packet is ```P2P_MAX_DATA_SIZE```. It is currently set to 60Bytes.

## Radio communication consideration
By default when using CRTP from a Crazyradio PA, for instance using the crazyflie-lib-python to communicate with a Crazyflie it will be sending pretty much non-stop by design. In order to give some air-time for P2P packets you should use the query `rate_limit=` when using P2P and Crazyradio PA at the same time.

Example, to limit the radio polling rate to 100Hz you would set the URI in a way similar to this:

```Python
URI = 'radio://0/80/2M/E7E7E7E7E7?rate_limit=100'

[...]

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
    [...]
```
## Using the P2P API
Functions and structures are defined in the header file `src/hal/interface/radiolink.h`. There is also an app layer example (`/app_peer_to_peer/`) available in the example folder of the repository.

## Peer to Peer broadcast

#### Sending P2P broadcast

From the STM, a packet can be sent using the `radiolinkSendP2PPacketBroadcast`` function. For example:

```c
static P2PPacket pk;
pk.port = 0;
pk.size = 11;
memcpy(pk.data, "Hello World", 11);
radiolinkSendP2PPacketBroadcast(&pk);
```

This function will trigger a single P2P packet to be transmitted by the radio.

#### Receiving P2P broadcast

If you want to receive packet in your function, you can register a callback with:

```c
p2pRegisterCB(p2pcallbackHandler);
```
    
with a function of the form of:
```c
void p2pcallbackHandler(P2PPacket *p)
{
  // p->port: P2P Port
  // p->size: Payload size
  // p->data: Payload data
  // p->rssi: RSSI of the packet. 40 means -40dBm.
}
```

The callback will be called by the task handling radio communication, it should then execute quickly (for example pushing data in a queue).



