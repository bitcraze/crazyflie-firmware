/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * TestTdoaStorage.c - Unit tests for tdoa storage functions
 */

// File under test
#include "tdoaStorage.h"

#include "unity.h"

#include <string.h>
#include "mock_clockCorrectionEngine.h"


#define TOF_VALIDITY_PERIOD (2 * 1000)
#define REMOTE_DATA_VALIDITY_PERIOD 30
#define ANCHOR_POSITION_VALIDITY_PERIOD (2 * 1000)


static tdaoAnchorInfoArray_t storage;
static void fixtureSetRemoteRxTime(tdoaAnchorContext_t* context, const uint8_t anchor, const uint32_t storageTime, const uint8_t remoteAnchor, const uint64_t remoteRxTime, const uint8_t seqNr);
static void fixtureSetTof(tdoaAnchorContext_t* context, const uint8_t anchor, const uint32_t storageTime, const uint8_t remoteAnchor, const uint64_t tof);

void setUp(void) {
  tdoaStorageInitialize(storage);
}

void testThatCurrentTimeIsSetInContextForGet() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t expectedTime = 123;

  // Test
  tdoaAnchorContext_t result;
  tdoaStorageGetAnchorCtx(storage, anchor, expectedTime, &result);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expectedTime, result.currentTime_ms);
}


void testThatCurrentTimeIsSetInContextForGetCreate() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t expectedTime = 123;

  // Test
  tdoaAnchorContext_t result;
  tdoaStorageGetCreateAnchorCtx(storage, anchor, expectedTime, &result);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expectedTime, result.currentTime_ms);
}


void testThatNoAnchorIsReturnedWhenItDoesNotExistInStorage() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t currentTime = 123;

  // Test
  tdoaAnchorContext_t result;
  bool actual = tdoaStorageGetAnchorCtx(storage, anchor, currentTime, &result);

  // Assert
  // False indicates that the anchor did not exist
  TEST_ASSERT_FALSE(actual);
  TEST_ASSERT_NULL(result.anchorInfo);
}


void testThatANewAnchorContextIsReturnedWhenItDoesNotExistInStorage() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t currentTime = 123;

  // Test
  tdoaAnchorContext_t result;
  bool actual = tdoaStorageGetCreateAnchorCtx(storage, anchor, currentTime, &result);

  // Assert
  // False indicates that the anchor did not exist
  TEST_ASSERT_FALSE(actual);
  TEST_ASSERT_NOT_NULL(result.anchorInfo);
}


void testThatTheSameAnchorContextIsReturnedWhenItAlreadyExistsInStorageForGet() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t currentTime = 123;

  // Make sure the anchor exists
  tdoaAnchorContext_t firstContext;
  tdoaStorageGetCreateAnchorCtx(storage, anchor, currentTime, &firstContext);

  // Test
  tdoaAnchorContext_t result;
  bool actual = tdoaStorageGetAnchorCtx(storage, anchor, currentTime, &result);

  // Assert
  // False indicates that the anchor did exist
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_PTR(firstContext.anchorInfo, result.anchorInfo);
}


void testThatTheSameAnchorContextIsReturnedWhenItAlreadyExistsInStorageForGetCreate() {
  // Fixture
  uint8_t anchor = 17;
  uint32_t currentTime = 123;

  // Make sure the anchor exists
  tdoaAnchorContext_t firstContext;
  tdoaStorageGetCreateAnchorCtx(storage, anchor, currentTime, &firstContext);

  // Test
  tdoaAnchorContext_t result;
  bool actual = tdoaStorageGetCreateAnchorCtx(storage, anchor, currentTime, &result);

  // Assert
  // False indicates that the anchor did exist
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_PTR(firstContext.anchorInfo, result.anchorInfo);
}


void testThatTheOldestAnchorContextIsReplacedWhenStorageIsFull() {
  // Fixture
  const uint32_t oldestAnchorTime = 900;
  const uint32_t baseAnchorTime = 1000;
  const uint32_t currentTime = 2000;

  uint8_t newAnchor = ANCHOR_STORAGE_COUNT;
  uint8_t oldestAnchor = 4;
  TEST_ASSERT_TRUE(oldestAnchor < ANCHOR_STORAGE_COUNT);

  // Make sure the storage is full with increasing update times, set the update
  // time for one slot to be oldest
  tdoaAnchorContext_t context;
  for (int id = 0; id < ANCHOR_STORAGE_COUNT; id++) {
    tdoaStorageGetCreateAnchorCtx(storage, id, currentTime, &context);

    uint32_t updateTime = baseAnchorTime + id;
    if (id == oldestAnchor) {
      updateTime = oldestAnchorTime;
    }

    context.currentTime_ms = updateTime;
    tdoaStorageSetRxTxData(&context, 0, 0, 0);
  }

  // Test
  tdoaAnchorContext_t result;
  bool actual = tdoaStorageGetCreateAnchorCtx(storage, newAnchor, currentTime, &result);

  // Assert
  TEST_ASSERT_FALSE(actual);
  TEST_ASSERT_TRUE(tdoaStorageIsAnchorInStorage(storage, newAnchor));
  TEST_ASSERT_FALSE(tdoaStorageIsAnchorInStorage(storage, oldestAnchor));
}


void testThatAListOfAnchorIdsIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  uint32_t currentTime = 1234;

  uint8_t expectedId0 = 17;
  uint8_t expectedId1 = 47;
  uint8_t expectedId2 = 11;

  uint8_t expectedCount = 3;

  tdoaStorageGetCreateAnchorCtx(storage, expectedId0, currentTime, &context);
  tdoaStorageGetCreateAnchorCtx(storage, expectedId1, currentTime, &context);
  tdoaStorageGetCreateAnchorCtx(storage, expectedId2, currentTime, &context);

  uint8_t unorderedAnchorList[10];

  // Test
  uint8_t actualCount = tdoaStorageGetListOfAnchorIds(storage, unorderedAnchorList, 10);

  // Assert
  TEST_ASSERT_EQUAL_INT8(expectedCount, actualCount);
  TEST_ASSERT_EQUAL_INT8(expectedId0, unorderedAnchorList[0]);
  TEST_ASSERT_EQUAL_INT8(expectedId1, unorderedAnchorList[1]);
  TEST_ASSERT_EQUAL_INT8(expectedId2, unorderedAnchorList[2]);
}

void testThatAListOfAnchorIdsIsReturnedButNotMoreThanTheListLength() {
  // Fixture
  tdoaAnchorContext_t context;
  uint32_t currentTime = 1234;

  uint8_t expectedId0 = 17;
  uint8_t expectedId1 = 47;
  uint8_t expectedId2 = 11;

  uint8_t expectedCount = 2;

  tdoaStorageGetCreateAnchorCtx(storage, expectedId0, currentTime, &context);
  tdoaStorageGetCreateAnchorCtx(storage, expectedId1, currentTime, &context);
  tdoaStorageGetCreateAnchorCtx(storage, expectedId2, currentTime, &context);

  uint8_t unorderedAnchorList[10];

  // Test
  uint8_t actualCount = tdoaStorageGetListOfAnchorIds(storage, unorderedAnchorList, expectedCount);

  // Assert
  TEST_ASSERT_EQUAL_INT8(expectedCount, actualCount);
  TEST_ASSERT_EQUAL_INT8(expectedId0, unorderedAnchorList[0]);
  TEST_ASSERT_EQUAL_INT8(expectedId1, unorderedAnchorList[1]);
}


void testThatAListOfActiveAnchorIdsIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  uint32_t oldTime = 1234; // Not valid
  // Validity time is 2000 ms
  uint32_t recentTime = 9000; // Still valid
  uint32_t currentTime = 10000;


  uint8_t expectedId0 = 17;
  uint8_t expectedId1 = 47;
  uint8_t otherId = 11;

  uint8_t expectedCount = 2;

  tdoaStorageGetCreateAnchorCtx(storage, otherId, oldTime, &context);
  tdoaStorageSetRxTxData(&context, 0, 0, 0);

  tdoaStorageGetCreateAnchorCtx(storage, expectedId0, recentTime, &context);
  tdoaStorageSetRxTxData(&context, 0, 0, 0);

  tdoaStorageGetCreateAnchorCtx(storage, expectedId1, recentTime, &context);
  tdoaStorageSetRxTxData(&context, 0, 0, 0);

  uint8_t unorderedAnchorList[10];

  // Test
  uint8_t actualCount = tdoaStorageGetListOfActiveAnchorIds(storage, unorderedAnchorList, 10, currentTime);

  // Assert
  TEST_ASSERT_EQUAL_INT8(expectedCount, actualCount);
  TEST_ASSERT_EQUAL_INT8(expectedId0, unorderedAnchorList[0]);
  TEST_ASSERT_EQUAL_INT8(expectedId1, unorderedAnchorList[1]);
}


void testThatAListOfActiveAnchorIdsIsReturnedButNotMoreThanTheListLength() {
  // Fixture
  tdoaAnchorContext_t context;
  uint32_t currentTime = 10000;


  uint8_t expectedId0 = 17;
  uint8_t otherId = 11;

  uint8_t expectedCount = 1;

  tdoaStorageGetCreateAnchorCtx(storage, expectedId0, currentTime, &context);
  tdoaStorageSetRxTxData(&context, 0, 0, 0);

  tdoaStorageGetCreateAnchorCtx(storage, otherId, currentTime, &context);
  tdoaStorageSetRxTxData(&context, 0, 0, 0);

  uint8_t unorderedAnchorList[10];

  // Test
  uint8_t actualCount = tdoaStorageGetListOfActiveAnchorIds(storage, unorderedAnchorList, expectedCount, currentTime);

  // Assert
  TEST_ASSERT_EQUAL_INT8(expectedCount, actualCount);
  TEST_ASSERT_EQUAL_INT8(expectedId0, unorderedAnchorList[0]);
}


void testThatAnchorPositionIsSetAndGet() {
  // Fixture
  float expectedX = 1.0;
  float expectedY = 2.0;
  float expectedZ = 3.0;
  uint32_t expectedTime = 1234;

  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, expectedTime, &context);

  tdoaStorageSetAnchorPosition(&context, expectedX, expectedY, expectedZ);

  uint32_t now = 2345;
  tdoaStorageGetAnchorCtx(storage, 0, now, &context);
  point_t actual;

  // Test
  bool isValid = tdoaStorageGetAnchorPosition(&context, &actual);

  // Assert
  TEST_ASSERT_TRUE(isValid);
  TEST_ASSERT_EQUAL_FLOAT(expectedX, actual.x);
  TEST_ASSERT_EQUAL_FLOAT(expectedY, actual.y);
  TEST_ASSERT_EQUAL_FLOAT(expectedZ, actual.z);
  TEST_ASSERT_EQUAL_UINT32(expectedTime, actual.timestamp);
}


void testThatAnchorPositionIsNotReturnedWhenTooOld() {
  // Fixture
  float x = 1.0;
  float y = 2.0;
  float z = 3.0;
  uint32_t now = 1234;

  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, now, &context);

  tdoaStorageSetAnchorPosition(&context, x, y, z);

  point_t voidResult;

  // Test
  context.currentTime_ms += ANCHOR_POSITION_VALIDITY_PERIOD;
  bool isValid = tdoaStorageGetAnchorPosition(&context, &voidResult);

  // Assert
  TEST_ASSERT_FALSE(isValid);
}


void testThatRxTxDataIsSet() {
  // Fixture
  uint32_t expectedUpdateTime = 1234;
  int64_t expectedRxTime = 4747474747;
  int64_t expectedTxTime = 1111111111;
  uint8_t expectedSeqNr = 17;

  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, expectedUpdateTime, &context);

  // Test
  tdoaStorageSetRxTxData(&context, expectedRxTime, expectedTxTime, expectedSeqNr);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(expectedUpdateTime, tdoaStorageGetLastUpdateTime(&context));
  TEST_ASSERT_EQUAL_UINT64(expectedRxTime, tdoaStorageGetRxTime(&context));
  TEST_ASSERT_EQUAL_UINT64(expectedTxTime, tdoaStorageGetTxTime(&context));
  TEST_ASSERT_EQUAL_UINT64(expectedSeqNr, tdoaStorageGetSeqNr(&context));
}


void testThatClockCorrectionIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  double expected = 123.456;
  clockCorrectionStorage_t* clockCorrectionStorage = tdoaStorageGetClockCorrectionStorage(&context);
  clockCorrectionEngineGet_ExpectAndReturn(clockCorrectionStorage, expected);

  // Test
  double actual = tdoaStorageGetClockCorrection(&context);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(expected, actual);
}


void testThatRemoteRxTimeIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  const uint8_t seqNr = 13;
  const uint8_t remoteAnchor = 17;
  const int64_t expectedRemoteRxTime = 4711;
  tdoaStorageSetRemoteRxTime(&context, remoteAnchor, expectedRemoteRxTime, seqNr);

  // Test
  int64_t actual = tdoaStorageGetRemoteRxTime(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expectedRemoteRxTime, actual);
}


void testThatRemoteRxTimeIsNotReturnedWhenOutdated() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint32_t storageTime = 1117;
  const uint32_t expiryTime = storageTime + REMOTE_DATA_VALIDITY_PERIOD;
  const uint8_t anchor = 0;


  const uint8_t seqNr = 13;
  const uint8_t remoteAnchor = 17;
  fixtureSetRemoteRxTime(&context, anchor, storageTime, remoteAnchor, 4711, seqNr);

  tdoaStorageGetCreateAnchorCtx(storage, anchor, expiryTime, &context);
  const int64_t expectedRemoteRxTime = 0;

  // Test
  int64_t actual = tdoaStorageGetRemoteRxTime(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expectedRemoteRxTime, actual);
}


void testThatRemoteRxTimeIsNotReturnedForUnknownRemoteAnchor() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);
  const uint8_t unkownRemoteAnchor = 17;
  const int64_t expectedRemoteRxTime = 0;

  // Test
  int64_t actual = tdoaStorageGetRemoteRxTime(&context, unkownRemoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expectedRemoteRxTime, actual);
}


void testThatRemoteRxTimeIsOverwrittenWhenSetWithTheSameRemoteId() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  const uint8_t seqNr = 13;
  const uint8_t remoteAnchor = 17;
  const uint32_t storageTime = 123;
  const uint8_t anchor = 3;

  const int64_t firstRemoteRxTime = 1234;
  fixtureSetRemoteRxTime(&context, anchor, storageTime, remoteAnchor, firstRemoteRxTime, seqNr);

  const int64_t expectedRemoteRxTime = 4711;
  fixtureSetRemoteRxTime(&context, anchor, storageTime, remoteAnchor, expectedRemoteRxTime, seqNr);

  // Test
  int64_t actual = tdoaStorageGetRemoteRxTime(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expectedRemoteRxTime, actual);
}


void testThatRemoteRxTimeReplacesTheOldestEntryWhenStorageIsFull() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 17;
  const uint8_t seqNr = 13;

  // Fill the storage (for one anchor) with remote data. Make sure one entry is
  // older than all the rest
  const uint8_t oldestRemoteAnchor = 3;
  TEST_ASSERT_TRUE(oldestRemoteAnchor < REMOTE_ANCHOR_DATA_COUNT);
  const uint32_t oldestStorageTime = 1117;

  const uint32_t newerStorageTime = oldestStorageTime + 1;

  const int64_t firstRemoteRxTime = 1234;
  for (int remoteAnchor = 0; remoteAnchor < REMOTE_ANCHOR_DATA_COUNT; remoteAnchor++) {
    if (remoteAnchor == oldestRemoteAnchor) {
      fixtureSetRemoteRxTime(&context, anchor, oldestStorageTime, remoteAnchor, firstRemoteRxTime, seqNr);
    } else {
      fixtureSetRemoteRxTime(&context, anchor, newerStorageTime, remoteAnchor, firstRemoteRxTime, seqNr);
    }
  }

  // Test
  // Set data for one more remote anchor that should replace the oldest in storage
  const uint8_t verificationRemoteAnchor = REMOTE_ANCHOR_DATA_COUNT;
  const uint64_t verificationRemoteRxTime = 6543;
  const uint32_t verificationStorageTime = newerStorageTime + 1;
  fixtureSetRemoteRxTime(&context, anchor, verificationStorageTime, verificationRemoteAnchor, verificationRemoteRxTime, seqNr);

  // Assert
  const int64_t actualVerification = tdoaStorageGetRemoteRxTime(&context, verificationRemoteAnchor);
  TEST_ASSERT_EQUAL_INT64(verificationRemoteRxTime, actualVerification);

  const int64_t actualReplaced = tdoaStorageGetRemoteRxTime(&context, oldestRemoteAnchor);
  TEST_ASSERT_EQUAL_INT64(0, actualReplaced);
}

void testThatRemoteRxTimeAndSequenceNumberIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  const uint8_t remoteAnchor = 17;
  const uint8_t expectedRemoteSeqNr = 13;
  const int64_t expectedRemoteRxTime = 4711;
  tdoaStorageSetRemoteRxTime(&context, remoteAnchor, expectedRemoteRxTime, expectedRemoteSeqNr);

  // Test
  int64_t actualRxTime = 0l;
  uint8_t actualSeqNr = 0;
  bool actual = tdoaStorageGetRemoteRxTimeSeqNr(&context, remoteAnchor, &actualRxTime, &actualSeqNr);

  // Assert
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_INT64(expectedRemoteRxTime, actualRxTime);
  TEST_ASSERT_EQUAL_INT8(expectedRemoteSeqNr, actualSeqNr);
}

void testThatRemoteRxTimeAndSequenceNumberIsNotReturnedWhenNotInList() {
  // Fixture
  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  const uint8_t remoteAnchor = 17;

  // Test
  int64_t actualRxTime = 0l;
  uint8_t actualSeqNr = 0;
  bool actual = tdoaStorageGetRemoteRxTimeSeqNr(&context, remoteAnchor, &actualRxTime, &actualSeqNr);

  // Assert
  TEST_ASSERT_FALSE(actual);
}

void testThatAListOfSequenceNumbersAndIdsOfRemoteAnchorsIsReturned() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 5;

  // Add remote data that is too old
  const uint8_t oldRemoteAncor = 3;
  const uint8_t oldSeqNr = 100;
  const uint32_t oldStorageTime = 1117;
  const uint64_t someRemoteRxTime = 1234;

  fixtureSetRemoteRxTime(&context, anchor, oldStorageTime, oldRemoteAncor, someRemoteRxTime, oldSeqNr);

  // Add remote data for two more anchors that are not too old
  const uint8_t activeRemoteAnchor0 = 11;
  const uint8_t activeSeqNr0 = 101;
  const uint8_t activeRemoteAnchor1 = 17;
  const uint8_t activeSeqNr1 = 102;
  const uint32_t activeStorageTime = oldStorageTime + 1;

  fixtureSetRemoteRxTime(&context, anchor, activeStorageTime, activeRemoteAnchor0, someRemoteRxTime, activeSeqNr0);
  fixtureSetRemoteRxTime(&context, anchor, activeStorageTime, activeRemoteAnchor1, someRemoteRxTime, activeSeqNr1);

  const uint32_t currentTime = oldStorageTime + REMOTE_DATA_VALIDITY_PERIOD;
  tdoaStorageGetCreateAnchorCtx(storage, anchor, currentTime, &context);

  int actualRemoteCount;
  uint8_t actualSequenceNumbers[REMOTE_ANCHOR_DATA_COUNT];
  uint8_t actualIds[REMOTE_ANCHOR_DATA_COUNT];

  // Test
  tdoaStorageGetRemoteSeqNrList(&context, &actualRemoteCount, actualSequenceNumbers, actualIds);

  // Assert
  TEST_ASSERT_EQUAL_INT32(2, actualRemoteCount);

  TEST_ASSERT_EQUAL_INT8(actualIds[0], activeRemoteAnchor0);
  TEST_ASSERT_EQUAL_INT8(actualSequenceNumbers[0], activeSeqNr0);

  TEST_ASSERT_EQUAL_INT8(actualIds[1], activeRemoteAnchor1);
  TEST_ASSERT_EQUAL_INT8(actualSequenceNumbers[1], activeSeqNr1);
}


void testThatNoTimeOfFlightIsReturnedWhenRemoteAnchorIsNotInStorage() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 5;
  const uint32_t storageTime = 1234;
  const uint8_t remoteAnchor = 17;
  const uint64_t expected = 0;

  tdoaStorageGetCreateAnchorCtx(storage, anchor, storageTime, &context);

  // Test
  int64_t actual = tdoaStorageGetRemoteTimeOfFlight(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expected, actual);
}


void testThatTimeOfFlightIsReturnedWhenSet() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 5;
  const uint32_t storageTime = 1234;
  const uint8_t remoteAnchor = 17;
  const uint64_t expected = 65432;

  fixtureSetTof(&context, anchor, storageTime, remoteAnchor, expected);

  // Test
  int64_t actual = tdoaStorageGetRemoteTimeOfFlight(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expected, actual);
}


void testThatTimeOfFlightIsReturnedWhenSetASecondTime() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 5;
  const uint32_t storageTime = 1234;
  const uint8_t remoteAnchor = 17;

  const uint64_t firstTof = 777777;
  const uint64_t expected = 65432;

  fixtureSetTof(&context, anchor, storageTime, remoteAnchor, firstTof);
  fixtureSetTof(&context, anchor, storageTime, remoteAnchor, expected);

  // Test
  int64_t actual = tdoaStorageGetRemoteTimeOfFlight(&context, remoteAnchor);

  // Assert
  TEST_ASSERT_EQUAL_INT64(expected, actual);
}


void testThatTofReplacesTheOldestEntryWhenStorageIsFull() {
  // Fixture
  tdoaAnchorContext_t context;
  const uint8_t anchor = 17;

  // Fill the storage (for one anchor) with Tofsa. Make sure one entry is
  // older than all the rest
  const uint8_t oldestRemoteAnchor = 3;
  TEST_ASSERT_TRUE(oldestRemoteAnchor < TOF_PER_ANCHOR_COUNT);
  const uint32_t oldestStorageTime = 1117;

  const uint32_t newerStorageTime = oldestStorageTime + 1;

  const int64_t firstTof = 1234;
  for (int remoteAnchor = 0; remoteAnchor < TOF_PER_ANCHOR_COUNT; remoteAnchor++) {
    if (remoteAnchor == oldestRemoteAnchor) {
      fixtureSetTof(&context, anchor, oldestStorageTime, remoteAnchor, firstTof);
    } else {
      fixtureSetTof(&context, anchor, newerStorageTime, remoteAnchor, firstTof);
    }
  }

  // Test
  // Set Tof for one more remote anchor that should replace the oldest in storage
  const uint8_t verificationRemoteAnchor = TOF_PER_ANCHOR_COUNT;
  const uint64_t verificationTof = 6543;
  const uint32_t verificationStorageTime = newerStorageTime + 1;
  fixtureSetTof(&context, anchor, verificationStorageTime, verificationRemoteAnchor, verificationTof);

  // Assert
  const int64_t actualVerification = tdoaStorageGetRemoteTimeOfFlight(&context, verificationRemoteAnchor);
  TEST_ASSERT_EQUAL_INT64(verificationTof, actualVerification);

  const int64_t actualReplaced = tdoaStorageGetRemoteTimeOfFlight(&context, oldestRemoteAnchor);
  TEST_ASSERT_EQUAL_INT64(0, actualReplaced);
}

// Not possible to put the ifdef outside the test function due to the way the test framework is implemented
void testThatTimeOfFlightIsSet() {
#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
  // Fixture
  uint32_t storageTime_ms = 1234;
  int64_t expectedToF = 4747474747;

  tdoaAnchorContext_t context;
  tdoaStorageGetCreateAnchorCtx(storage, 0, 0, &context);

  // Test
  tdoaStorageSetTimeOfFlight(&context, expectedToF, storageTime_ms);

  // Assert
  TEST_ASSERT_EQUAL_UINT64(expectedToF, tdoaStorageGetTimeOfFlight(&context, storageTime_ms - 1));
  TEST_ASSERT_EQUAL_UINT64(expectedToF, tdoaStorageGetTimeOfFlight(&context, storageTime_ms));
  TEST_ASSERT_EQUAL_UINT64(0, tdoaStorageGetTimeOfFlight(&context, storageTime_ms + 1));
#endif
}


// Helpers ///////////////

static void fixtureSetRemoteRxTime(tdoaAnchorContext_t* context, const uint8_t anchor, const uint32_t storageTime, const uint8_t remoteAnchor, const uint64_t remoteRxTime, const uint8_t seqNr) {
  tdoaStorageGetCreateAnchorCtx(storage, anchor, storageTime, context);
  tdoaStorageSetRemoteRxTime(context, remoteAnchor, remoteRxTime, seqNr);
}

static void fixtureSetTof(tdoaAnchorContext_t* context, const uint8_t anchor, const uint32_t storageTime, const uint8_t remoteAnchor, const uint64_t tof) {
  tdoaStorageGetCreateAnchorCtx(storage, anchor, storageTime, context);
  tdoaStorageSetRemoteTimeOfFlight(context, remoteAnchor, tof);
}
