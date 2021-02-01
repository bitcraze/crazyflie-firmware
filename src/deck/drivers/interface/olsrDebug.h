#ifndef __OLSR_DEBUG_H__
#define __OLSR_DEBUG_H__

#include "debug.h"

/* Instead of using DEBUG_PRINT, one should use these function
 *     DEBUG_PRINT_OLSR_SYSTEM
 *     DEBUG_PRINT_OLSR_SEND
 *     DEBUG_PRINT_OLSR_RECEIVE
 *     DEBUG_PRINT_OLSR_HELLO
 *     DEBUG_PRINT_OLSR_TC
 *     DEBUG_PRINT_OLSR_TS
 *     DEBUG_PRINT_OLSR_LINK
 *     DEBUG_PRINT_OLSR_MPR
 *     DEBUG_PRINT_OLSR_MS
 *     DEBUG_PRINT_OLSR_NEIGHBOR
 *     DEBUG_PRINT_OLSR_NEIGHBOR2
 *     DEBUG_PRINT_OLSR_DUPLICATE
 *     DEBUG_PRINT_OLSR_TOPOLOGY
 *     DEBUG_PRINT_OLSR_ROUTING 
 *     DEBUG_PRINT_OLSR_SET
 *     DEBUG_PRINT_OLSR_FORWARD
 * There debug printing can be switched on or off by #define DEBUG_OLSR_XXXXX
 */

//#define DEBUG_OLSR_SYSTEM
#define DEBUG_OLSR_SEND
#define DEBUG_OLSR_RECEIVE
#define DEBUG_OLSR_HELLO
#define DEBUG_OLSR_TC
#define DEBUG_OLSR_TS
#define DEBUG_OLSR_LINK
#define DEBUG_OLSR_MPR
#define DEBUG_OLSR_MS
#define DEBUG_OLSR_NEIGHBOR
#define DEBUG_OLSR_NEIGHBOR2
#define DEBUG_OLSR_DUPLICATE
#define DEBUG_OLSR_TOPOLOGY
#define DEBUG_OLSR_ROUTING
#define DEBUG_OLSR_SET
#define DEBUG_OLSR_FORWARD
#define DEBUG_OLSR_APP
//#define DEBUG_OLSR_SIM

#ifdef DEBUG_OLSR_SYSTEM
#define DEBUG_PRINT_OLSR_SYSTEM(fmt, ...) DEBUG_PRINT("SYSTEM: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_SYSTEM(fmt, ...)
#endif

#ifdef DEBUG_OLSR_SIM
#define DEBUG_PRINT_OLSR_SIM(fmt, ...) DEBUG_PRINT("SIM: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_SIM(fmt, ...)
#endif

#ifdef DEBUG_OLSR_SEND
#define DEBUG_PRINT_OLSR_SEND(fmt, ...) DEBUG_PRINT("SEND: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_SEND(fmt, ...)
#endif


#ifdef DEBUG_OLSR_RECEIVE
#define DEBUG_PRINT_OLSR_RECEIVE(fmt, ...) DEBUG_PRINT("RECEIVE: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_RECEIVE(fmt, ...)
#endif

#ifdef DEBUG_OLSR_APP
#define DEBUG_PRINT_OLSR_APP(fmt, ...) DEBUG_PRINT("APP: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_APP(fmt, ...)
#endif

#ifdef DEBUG_OLSR_HELLO
#define DEBUG_PRINT_OLSR_HELLO(fmt, ...) DEBUG_PRINT("HELLO: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_HELLO(fmt, ...)
#endif

#ifdef DEBUG_OLSR_TC
#define DEBUG_PRINT_OLSR_TC(fmt, ...) DEBUG_PRINT("TC: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_TC(fmt, ...)
#endif

#ifdef DEBUG_OLSR_TS
#define DEBUG_PRINT_OLSR_TS(fmt, ...) DEBUG_PRINT("TS: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_TS(fmt, ...)
#endif

#ifdef DEBUG_OLSR_LINK
#define DEBUG_PRINT_OLSR_LINK(fmt, ...) DEBUG_PRINT("LINK: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_LINK(fmt, ...)
#endif

#ifdef DEBUG_OLSR_MPR
#define DEBUG_PRINT_OLSR_MPR(fmt, ...) DEBUG_PRINT("MPR: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_MPR(fmt, ...)
#endif

#ifdef DEBUG_OLSR_MS
#define DEBUG_PRINT_OLSR_MS(fmt, ...) DEBUG_PRINT("MS: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_MS(fmt, ...)
#endif

#ifdef DEBUG_OLSR_NEIGHBOR
#define DEBUG_PRINT_OLSR_NEIGHBOR(fmt, ...) DEBUG_PRINT("NEIGHBOR: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_NEIGHBOR(fmt, ...)
#endif

#ifdef DEBUG_OLSR_NEIGHBOR2
#define DEBUG_PRINT_OLSR_NEIGHBOR2(fmt, ...) DEBUG_PRINT("NEIGHBOR2: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_NEIGHBOR2(fmt, ...)
#endif

#ifdef DEBUG_OLSR_DUPLICATE
#define DEBUG_PRINT_OLSR_DUPLICATE(fmt, ...) DEBUG_PRINT("DUPLICATE: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_DUPLICATE(fmt, ...)
#endif

#ifdef DEBUG_OLSR_TOPOLOGY
#define DEBUG_PRINT_OLSR_TOPOLOGY(fmt, ...) DEBUG_PRINT("TOPOLOGY: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_TOPOLOGY(fmt, ...)
#endif

#ifdef DEBUG_OLSR_ROUTING
#define DEBUG_PRINT_OLSR_ROUTING(fmt, ...) DEBUG_PRINT("ROUTING: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_ROUTING(fmt, ...)
#endif

#ifdef DEBUG_OLSR_SET
#define DEBUG_PRINT_OLSR_SET(fmt, ...) DEBUG_PRINT("SET: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_SET(fmt, ...)
#endif

#ifdef DEBUG_OLSR_FORWARD
#define DEBUG_PRINT_OLSR_FORWARD(fmt, ...) DEBUG_PRINT("FORWARD: "fmt,  ##__VA_ARGS__)
#else
#define DEBUG_PRINT_OLSR_FORWARD(fmt, ...)
#endif

#endif //__OLSR_DEBUG_H__
