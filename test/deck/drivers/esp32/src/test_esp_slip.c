// File under test esp_slip.crftq
#include "esp_slip.h"

#include <string.h>

#include "unity.h"

// Mocks
static void mockPutChar(uint32_t size, uint8_t *data);
static bool mockGetChar(uint8_t *c, const uint32_t timeoutTicks);

static uint8_t mockPutCharBuf[256];
static int mockPutCharBufSize = 0;
static uint8_t mockGetCharBuf[256];
static int mockGetCharBufIndex = 0;
static int mockGetCharDirtCounter = 0;
static bool mockGetCharHasReceivedSend = false;

// Helpers
static void setupSenderHeader(espSlipSendPacket_t *senderPckt, const uint8_t command);
static void setupEmptyReceivePacket(espSlipReceivePacket_t *receiverPckt, const uint8_t command);

void setUp(void)
{
  // emptyResultBuffer
  mockPutCharBufSize = 0;
  memset(mockPutCharBuf, 0, sizeof(mockPutCharBuf));
  mockGetCharBufIndex = 0;
  memset(mockGetCharBuf, 0, sizeof(mockGetCharBuf));
  mockGetCharDirtCounter = 0;
  mockGetCharHasReceivedSend = false;
}

void testFullSentPacket()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  senderPckt.command = command;
  senderPckt.dataSize = 0x03;
  uint8_t sendBuffer[14];
  sendBuffer[9] = 6;
  sendBuffer[10] = 7;
  sendBuffer[11] = 8;

  espSlipReceivePacket_t receiverPckt;
  setupEmptyReceivePacket(&receiverPckt, command);

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_HEX8(0xC0, mockPutCharBuf[0]);    // start byte
  TEST_ASSERT_EQUAL_HEX8(0x00, mockPutCharBuf[1]);    // direction
  TEST_ASSERT_EQUAL_HEX8(command, mockPutCharBuf[2]); // command
  TEST_ASSERT_EQUAL_HEX8(0x03, mockPutCharBuf[3]);    // data payload size
  TEST_ASSERT_EQUAL_HEX8(0x00, mockPutCharBuf[4]);    // data payload size
  TEST_ASSERT_EQUAL_HEX8(0x06, mockPutCharBuf[9]);
  TEST_ASSERT_EQUAL_HEX8(0x07, mockPutCharBuf[10]);
  TEST_ASSERT_EQUAL_HEX8(0x08, mockPutCharBuf[11]);
  TEST_ASSERT_EQUAL_HEX8(0xC0, mockPutCharBuf[12]); // end byte
}

void testEscapedSendData()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  senderPckt.command = command;
  senderPckt.dataSize = 0x04;
  uint8_t sendBuffer[14];
  sendBuffer[9] = 6;
  sendBuffer[10] = 0xC0;
  sendBuffer[11] = 0xDB;
  sendBuffer[12] = 7;

  espSlipReceivePacket_t receiverPckt;
  setupEmptyReceivePacket(&receiverPckt, command);

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_HEX8(0xC0, mockPutCharBuf[0]); // start byte
  TEST_ASSERT_EQUAL_HEX8(0x04, mockPutCharBuf[3]); // data payload size
  TEST_ASSERT_EQUAL_HEX8(0x00, mockPutCharBuf[4]); // data payload size
  TEST_ASSERT_EQUAL_HEX8(0x06, mockPutCharBuf[9]);
  TEST_ASSERT_EQUAL_HEX8(0xDB, mockPutCharBuf[10]);
  TEST_ASSERT_EQUAL_HEX8(0xDC, mockPutCharBuf[11]);
  TEST_ASSERT_EQUAL_HEX8(0xDB, mockPutCharBuf[12]);
  TEST_ASSERT_EQUAL_HEX8(0xDD, mockPutCharBuf[13]);
  TEST_ASSERT_EQUAL_HEX8(0x07, mockPutCharBuf[14]);
}

void testExtractCorrectReceivePcktCommands()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);
  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;

  mockGetCharBuf[0] = 0xC0;    // start byte
  mockGetCharBuf[1] = 0x01;    // direction
  mockGetCharBuf[2] = command; // command
  mockGetCharBuf[3] = 0x04;    // data payload size
  mockGetCharBuf[4] = 0x00;    // data payload size
  mockGetCharBuf[9] = 0x00;    // status byte
  mockGetCharBuf[10] = 0x00;
  mockGetCharBuf[11] = 0x00;
  mockGetCharBuf[12] = 0x00;
  mockGetCharBuf[13] = 0xC0; // end byte

  // Test
  bool actual = espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_INT8(0x17, receiverPckt.command);
  TEST_ASSERT_TRUE(actual);
}

void testReceivedData()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);

  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;

  mockGetCharBuf[0] = 0xC0;    // start byte
  mockGetCharBuf[1] = 0x01;    // direction
  mockGetCharBuf[2] = command; // command
  mockGetCharBuf[3] = 0x09;    // data payload size
  mockGetCharBuf[4] = 0x00;    // data payload size
  mockGetCharBuf[9] = 0x50;
  mockGetCharBuf[10] = 0x12;
  mockGetCharBuf[11] = 0x17;
  mockGetCharBuf[12] = 0x55;
  mockGetCharBuf[13] = 0x03;
  mockGetCharBuf[14] = 0x00; // status byte
  mockGetCharBuf[15] = 0x00;
  mockGetCharBuf[16] = 0x00;
  mockGetCharBuf[17] = 0x00;
  mockGetCharBuf[18] = 0xC0; // end byte

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_HEX8(0x50, receiverPckt.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x12, receiverPckt.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x17, receiverPckt.data[2]);
  TEST_ASSERT_EQUAL_HEX8(0x55, receiverPckt.data[3]);
  TEST_ASSERT_EQUAL_HEX8(0x03, receiverPckt.data[4]);
}

void testEscapedReceivedData()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);

  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;
  mockGetCharBuf[0] = 0xC0;    // start byte
  mockGetCharBuf[1] = 0x01;    // direction
  mockGetCharBuf[2] = command; // command
  mockGetCharBuf[3] = 0x07;    // data payload size
  mockGetCharBuf[4] = 0x00;    // data payload size
  mockGetCharBuf[9] = 0xDB;    // data payload
  mockGetCharBuf[10] = 0xDC;
  mockGetCharBuf[11] = 0xDB;
  mockGetCharBuf[12] = 0xDD;
  mockGetCharBuf[13] = 0x03;
  mockGetCharBuf[14] = 0x00; // status byte
  mockGetCharBuf[15] = 0x00;
  mockGetCharBuf[16] = 0x00;
  mockGetCharBuf[17] = 0x00;
  mockGetCharBuf[18] = 0xC0; // end byte

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_HEX8(0xC0, receiverPckt.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0xDB, receiverPckt.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x03, receiverPckt.data[2]);
}

void testWrongCommandRaisesError()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);

  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;

  mockGetCharBuf[0] = 0xC0;  // start byte
  mockGetCharBuf[1] = 0x01;  // direction
  mockGetCharBuf[2] = 0x29;  // wrong command
  mockGetCharBuf[3] = 0x04;  // data payload size
  mockGetCharBuf[4] = 0x00;  // data payload size
  mockGetCharBuf[14] = 0x00; // status byte
  mockGetCharBuf[15] = 0x00;
  mockGetCharBuf[16] = 0x00;
  mockGetCharBuf[17] = 0x00;
  mockGetCharBuf[18] = 0xC0; // end byte

  // Test
  bool actual = espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_FALSE(actual);
}

void testThatWeIgnoreLeadingGarbage()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);

  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;
  mockGetCharBuf[0] = 0x23;    // garbage
  mockGetCharBuf[1] = 0x34;    // garbage
  mockGetCharBuf[2] = 0x45;    // garbage
  mockGetCharBuf[3] = 0xC0;    // start byte
  mockGetCharBuf[4] = 0x01;    // direction
  mockGetCharBuf[5] = command; // command
  mockGetCharBuf[6] = 0x09;    // data payload size
  mockGetCharBuf[7] = 0x00;    // data payload size
  mockGetCharBuf[12] = 0x50;
  mockGetCharBuf[13] = 0x12;
  mockGetCharBuf[14] = 0x17;
  mockGetCharBuf[15] = 0x55;
  mockGetCharBuf[16] = 0x03;
  mockGetCharBuf[17] = 0x00; // status byte
  mockGetCharBuf[18] = 0x00;
  mockGetCharBuf[19] = 0x00;
  mockGetCharBuf[20] = 0x00;
  mockGetCharBuf[21] = 0xC0; // end byte

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_HEX8(0x50, receiverPckt.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x12, receiverPckt.data[1]);
  TEST_ASSERT_EQUAL_HEX8(0x17, receiverPckt.data[2]);
  TEST_ASSERT_EQUAL_HEX8(0x55, receiverPckt.data[3]);
  TEST_ASSERT_EQUAL_HEX8(0x03, receiverPckt.data[4]);
}

void testThatWeDetectMissingEndMarker()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  setupSenderHeader(&senderPckt, command);

  uint8_t sendBuffer[14];

  espSlipReceivePacket_t receiverPckt;
  mockGetCharBuf[0] = 0xC0;    // start byte
  mockGetCharBuf[1] = 0x01;    // direction
  mockGetCharBuf[2] = command; // command
  mockGetCharBuf[3] = 0x07;    // data payload size
  mockGetCharBuf[4] = 0x00;    // data payload size
  mockGetCharBuf[9] = 0xDB;    // data payload
  mockGetCharBuf[10] = 0xDC;
  mockGetCharBuf[11] = 0xDB;
  mockGetCharBuf[12] = 0xDD;
  mockGetCharBuf[13] = 0x03;
  mockGetCharBuf[14] = 0x00; // status byte
  mockGetCharBuf[15] = 0x00;
  mockGetCharBuf[16] = 0x00;
  mockGetCharBuf[17] = 0x00;
  mockGetCharBuf[18] = 0x00; // Missing end marker

  // Test
  bool actual = espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_FALSE(actual);
}

void testThatTheBufferIsClearedBeforeWritingSomething()
{
  // Fixture
  const uint8_t command = 0x17;

  espSlipSendPacket_t senderPckt;
  senderPckt.command = command;
  senderPckt.dataSize = 0x03;

  uint8_t sendBuffer[14];
  sendBuffer[9] = 6;
  sendBuffer[10] = 7;
  sendBuffer[11] = 8;

  espSlipReceivePacket_t receiverPckt;
  setupEmptyReceivePacket(&receiverPckt, command);
  mockGetCharDirtCounter = 10;

  // Test
  espSlipExchange(&sendBuffer[0], &receiverPckt, &senderPckt, mockPutChar, mockGetChar, 100);

  // Assert
  TEST_ASSERT_EQUAL_INT32(0, mockGetCharDirtCounter);
}

// Helpers ///////////////////////////////////////////////////////////////////////////////////////////////

static void mockPutChar(uint32_t size, uint8_t *data)
{
  mockGetCharHasReceivedSend = true;
  memcpy(&mockPutCharBuf[mockPutCharBufSize], data, size);
  mockPutCharBufSize += size;
}

static bool mockGetChar(uint8_t *c, const uint32_t timeoutTicks)
{
  if (mockGetCharHasReceivedSend)
  {
    *c = mockGetCharBuf[mockGetCharBufIndex++];

    return true;
  }
  else
  {
    if (mockGetCharDirtCounter == 0)
    {
      return false;
    }

    // Return start of packet as garbage to make sure the state machine failes if it is red
    *c = 0xC0;

    mockGetCharDirtCounter--;
    return true;
  }
}

static void setupSenderHeader(espSlipSendPacket_t *senderPckt, const uint8_t command)
{
  senderPckt->command = command;
  senderPckt->dataSize = 0x04;
}

static void setupEmptyReceivePacket(espSlipReceivePacket_t *receiverPckt, const uint8_t command)
{
  mockGetCharBuf[0] = 0xC0;    // start byte
  mockGetCharBuf[1] = 0x01;    // direction
  mockGetCharBuf[2] = command; // command
  mockGetCharBuf[3] = 0x04;    // data payload size
  mockGetCharBuf[4] = 0x00;    // data payload size
  mockGetCharBuf[9] = 0x00;    // status byte
  mockGetCharBuf[10] = 0x00;
  mockGetCharBuf[11] = 0x00;
  mockGetCharBuf[12] = 0x00;
  mockGetCharBuf[13] = 0xC0; // end byte
}
