#include "gtest/gtest.h"

#include "Arduino.h"

#include "../fake/fastRW.h"

#include "IOLogic.h"


using ::testing::Return;

static unsigned int pinMap[10] = { 8, 11, 12, 7, 21, 1, 0, 0, 0, 0 };

TEST(IOLogicTest, setOutput) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalWrite(7, HIGH));
    EXPECT_CALL(*arduinoMock, pinMode(pinMap[3], OUTPUT));

    IOLogic iol(pinMap);
    iol.setOutput(3, 1);
    releaseArduinoMock();
}

TEST(IOLogicTest, waitForInput) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(pinMap[4])).WillOnce(Return(1));
    EXPECT_CALL(*arduinoMock, pinMode(pinMap[4], INPUT_PULLDOWN));

    IOLogic iol(pinMap);

    iol.addCondition(4, IOLogic::IO_HIGH);
    EXPECT_EQ(iol.isDone(),true);
    releaseArduinoMock();
}

TEST(IOLogicTest, waitForInputIsNotDone) {
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(pinMap[4])).WillOnce(Return(1));
    EXPECT_CALL(*arduinoMock, pinMode(pinMap[4], INPUT_PULLUP));

    IOLogic iol(pinMap);

    iol.addCondition(4, IOLogic::IO_LOW);
    EXPECT_EQ(iol.isDone(),false);
    releaseArduinoMock();
}
