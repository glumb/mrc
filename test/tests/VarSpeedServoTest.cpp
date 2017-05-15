#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Arduino.h"

#include "../fake/Servo.h"

#include "VarSpeedServo.h"



TEST(VarSpeedServoTest, testPinNumber)
{
    VarSpeedServo S(1, 2, 3, 4, 5, 6);

    EXPECT_EQ(S.getPinNumber(), 1);
}

TEST(VarSpeedServoTest, maxAngle)
{
    VarSpeedServo S(1, 2, 3, 4, 5, 6);

    EXPECT_EQ(S.getMaxAngleVelocity(), 2);
}
