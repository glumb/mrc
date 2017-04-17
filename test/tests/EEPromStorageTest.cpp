#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "EEPROM.h"
#include "Arduino.h"

#include "../src/EEPromStorage.h"

using ::testing::Return;

TEST(EEPromStorage, clear) {
    EEPROMMock *mock     = EEPROMMockInstance();
    int expected_address = 0;
    int expected_value   = 0;

    EXPECT_CALL(*mock, write(expected_address, expected_value));

    // EXPECT_CALL(*mock, read(expected_value));

    EEPromStorage storage;
    storage.clear();

    releaseEEPROMMock();
}

TEST(EEPromStorage, GetMessage) {
    EEPROMMock *mock = EEPROMMockInstance();


    EXPECT_CALL(*mock, read(0));

    EEPromStorage storage;

    char message[10];
    storage.getMessage(0, message);


    releaseEEPROMMock();
}
