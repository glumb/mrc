#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Arduino.h"

#include "../fake/Servo.h"

#include "MyServo.h"



TEST(MyServoTest, testPinNumber)
{
    MyServo S(1, 2, 3, 4, 5, 6);

    EXPECT_EQ(S.getPinNumber(), 1);
}

TEST(MyServoTest, maxAngle)
{
    MyServo S(1, 2, 3, 4, 5, 6);

    EXPECT_EQ(S.getMaxAngleVelocity(), 2);
}
