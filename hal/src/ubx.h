#ifndef __UBX_H__
#define __UBX_H__

struct aid_alpsrv {
    uint8_t idSize;
    uint8_t type;
    uint16_t ofs;
    uint16_t size;
    uint16_t fileId;
    uint16_t dataSize;
    uint8_t id1;
    uint8_t id2;
    uint32_t id3;
} __attribute__ ((packed));

struct nav_posllh {
    uint32_t itow;
    int32_t lon;
    int32_t lat;
    int32_t height;
    uint32_t hacc;
    uint32_t vacc;
} __attribute__ ((packed));
  
struct nav_pvt {
    uint32_t iTow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t reserved1;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint16_t reserved2;
    uint32_t reserved3;
} __attribute__ ((packed));

struct ubx_message {
    union {
        struct {
            uint8_t class;
            uint8_t id;
        } __attribute__ ((packed));
        uint16_t class_id;
    } __attribute__ ((packed));
    uint16_t len;
    union {
        void * payload;
        struct nav_pvt* nav_pvt;
        struct nav_posllh* nav_posllh;
        struct aid_alpsrv* naid_alpsrv;
    };
    union {
        struct {
            uint8_t ck_a;
            uint8_t ck_b;
        } __attribute__ ((packed));
        uint16_t ck;
    };
} __attribute__ ((packed));

#define CFG_MSG 0x0106
#define AID_ALPSRV 0x320b

#define NAV_POSLLH 0x0201
#define NAV_PVT 0x0701

#endif //__UBX_H__
