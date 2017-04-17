#include "gtest/gtest.h"

#include "Arduino.h"

#include "../fake/fastRW.h"

#include "IOLogic.h"


using ::testing::Return;

// somehow expect pin to be set

TEST(IOLogicTest, setOutput) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalWrite(8, HIGH));
    EXPECT_CALL(*arduinoMock, pinMode(8, OUTPUT));

    IOLogic iol;
    iol.setOutput(8, 1);
    releaseArduinoMock();
}

TEST(delay, waitForInput) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(6)).WillOnce(Return(1));
    EXPECT_CALL(*arduinoMock, pinMode(6, INPUT));

    IOLogic iol;

    iol.addCondition(6, IOLogic::IO_HIGH);
    EXPECT_EQ(iol.isDone(),true);
    releaseArduinoMock();
}

TEST(delay, waitForInputIsNotDone) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(6)).WillOnce(Return(1));
    EXPECT_CALL(*arduinoMock, pinMode(6, INPUT));

    IOLogic iol;

    iol.addCondition(6, IOLogic::IO_LOW);
    EXPECT_EQ(iol.isDone(),false);
    releaseArduinoMock();
}
