#ifndef MOCK_MY_SERVO_H
#define MOCK_MY_SERVO_H 1

#include "MyServo.h"

class MockMyServo : public MyServo
{
public:
    MOCK_METHOD1(setTargetRadAngle,    void(float radAngle));
    MOCK_METHOD0(getPinNumber,    int());
};

#endif /* ifndef MY_SERVO_H */
