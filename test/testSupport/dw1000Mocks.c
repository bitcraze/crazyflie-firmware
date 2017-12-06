#include "dw1000Mocks.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"


// dwGetData mock /////////////////////////////////////////////////////////////

#define DW_GET_DATA_MAX_CALLS 100

static struct {
  dwDevice_t* expectedDev;
  unsigned int expectedDataLength;
  packet_t packet;
} dwGetDataContexts[DW_GET_DATA_MAX_CALLS];

static int dwGetDataMockCallIndex = 0;

static void dwGetDataMockCallback(dwDevice_t* actualDev, uint8_t* data, unsigned int actualDataLength, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetDataContexts[cmock_num_calls].expectedDev, actualDev);
  TEST_ASSERT_EQUAL_UINT(dwGetDataContexts[cmock_num_calls].expectedDataLength, actualDataLength);

  memcpy(data, &dwGetDataContexts[cmock_num_calls].packet, actualDataLength);
}

void dwGetData_ExpectAndCopyData(dwDevice_t* expDev, const packet_t* rxPacket, unsigned int expDataLength) {
  TEST_ASSERT_TRUE(dwGetDataMockCallIndex < DW_GET_DATA_MAX_CALLS);

  dwGetDataContexts[dwGetDataMockCallIndex].expectedDev = expDev;
  dwGetDataContexts[dwGetDataMockCallIndex].expectedDataLength = expDataLength;
  memcpy(&dwGetDataContexts[dwGetDataMockCallIndex].packet, rxPacket, sizeof(packet_t));

  dwGetData_StubWithCallback(dwGetDataMockCallback);

  dwGetDataMockCallIndex++;
}

void dwGetData_resetMock() {
  dwGetDataMockCallIndex = 0;
}


// dwGetTransmitTimestamp mock ////////////////////////////////////////////////

#define DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS 100

static struct {
  dwDevice_t* expectedDev;
  dwTime_t time;
} dwGetTransmitTimestampContexts[DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS];

static int dwGetTransmitTimestampMockCallIndex = 0;

static void dwGetTransmitTimestampMockCallback(dwDevice_t* actualDev, dwTime_t* time, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetTransmitTimestampContexts[cmock_num_calls].expectedDev, actualDev);
  memcpy(time, &dwGetTransmitTimestampContexts[cmock_num_calls].time, sizeof(dwTime_t));
}

void dwGetTransmitTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time) {
  TEST_ASSERT_TRUE(dwGetTransmitTimestampMockCallIndex < DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS);

  dwGetTransmitTimestampContexts[dwGetTransmitTimestampMockCallIndex].expectedDev = expDev;
  memcpy(&dwGetTransmitTimestampContexts[dwGetTransmitTimestampMockCallIndex].time, time, sizeof(dwTime_t));

  dwGetTransmitTimestamp_StubWithCallback(dwGetTransmitTimestampMockCallback);

  dwGetTransmitTimestampMockCallIndex++;
}

void dwGetTransmitTimestamp_resetMock() {
  dwGetTransmitTimestampMockCallIndex = 0;
}


// dwGetReceiveTimestamp mock /////////////////////////////////////////////////

#define DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS 100

static struct {
  dwDevice_t* expectedDev;
  dwTime_t time;
} dwGetReceiveTimestampContexts[DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS];

int dwGetReceiveTimestampMockCallIndex = 0;

static void dwGetReceiveTimestampMockCallback(dwDevice_t* actualDev, dwTime_t* time, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetReceiveTimestampContexts[cmock_num_calls].expectedDev, actualDev);
  memcpy(time, &dwGetReceiveTimestampContexts[cmock_num_calls].time, sizeof(dwTime_t));
}

void dwGetReceiveTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time) {
  TEST_ASSERT_TRUE(dwGetReceiveTimestampMockCallIndex < DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS);

  dwGetReceiveTimestampContexts[dwGetReceiveTimestampMockCallIndex].expectedDev = expDev;
  memcpy(&dwGetReceiveTimestampContexts[dwGetReceiveTimestampMockCallIndex].time, time, sizeof(dwTime_t));

  dwGetReceiveTimestamp_StubWithCallback(dwGetReceiveTimestampMockCallback);

  dwGetReceiveTimestampMockCallIndex++;
}

void dwGetReceiveTimestamp_resetMock() {
  dwGetReceiveTimestampMockCallIndex = 0;
}
