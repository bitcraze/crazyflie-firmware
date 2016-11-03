#ifndef __DW_1000_MOCKS_H__
#define __DW_1000_MOCKS_H__

#include "libdw1000Types.h"
#include "mac.h"

void dwGetData_ExpectAndCopyData(dwDevice_t* expDev, const packet_t* rxPacket, unsigned int expDataLength);
void dwGetData_resetMock();

void dwGetTransmitTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);
void dwGetTransmitTimestamp_resetMock();

void dwGetReceiveTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);
void dwGetReceiveTimestamp_resetMock();

#endif // __DW_1000_MOCKS_H__
