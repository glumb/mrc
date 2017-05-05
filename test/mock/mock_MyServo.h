#ifndef MOCK_MY_SERVO_H
#define MOCK_MY_SERVO_H 1

#include "MyServo.h"
#include "gmock/gmock.h"

class mock_MyServo : public MyServo {
public:

    mock_MyServo(int          pinNumber,
                 float        maxAngleVelocity,
                 unsigned int minFreq,
                 unsigned int maxFreq,
                 float        minRadAngle,
                 float        maxRadAngle,
                 float        homeRadAngle) :
        MyServo(pinNumber,
                maxAngleVelocity,
                minFreq,
                maxFreq,
                minRadAngle,
                maxRadAngle,
                homeRadAngle) {}

    mock_MyServo() :
        MyServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0) {}

    MOCK_METHOD1(setTargetRadAngle, void(float radAngle));
    MOCK_METHOD0(getCurrentAngle, float());
    MOCK_METHOD0(getPinNumber,    int());
};

#endif /* ifndef MOCK_MY_SERVO_H */
