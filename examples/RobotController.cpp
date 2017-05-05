#include <Arduino.h>
#include "../src/RobotController.h"

#define pin_servo_0 0
#define pin_servo_1 1
#define pin_servo_2 2
#define pin_servo_3 3
#define pin_servo_4 4
#define pin_servo_5 5

// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
const float servoConfig[6][7] = {
    { pin_servo_0, 160.00 * DEG_TO_RAD,   700.00, 2380.00,  -90.00 * DEG_TO_RAD,   90.00 * DEG_TO_RAD, 0 },
    { pin_servo_1, 160.00 * DEG_TO_RAD,  1880.00,  710.00,  -90.00 * DEG_TO_RAD,   45.00 * DEG_TO_RAD, 0 },
    { pin_servo_2, 160.00 * DEG_TO_RAD,   640.00, 2330.00, -135.00 * DEG_TO_RAD,   45.00 * DEG_TO_RAD, 0 },
    { pin_servo_3, 160.00 * DEG_TO_RAD,   740.00, 2260.00,  -90.00 * DEG_TO_RAD,   85.00 * DEG_TO_RAD, 0 },
    { pin_servo_4, 160.00 * DEG_TO_RAD,  2360.00,  730.00,  -15.00 * DEG_TO_RAD,  140.00 * DEG_TO_RAD, 0 },
    { pin_servo_5, 160.00 * DEG_TO_RAD,   740.00, 2200.00,  -90.00 * DEG_TO_RAD,   60.00 * DEG_TO_RAD, 0 }
};

float logicalAngleLimits[6][2] = {
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD }
};

float geometry[5][3] = {
    {    5, 0,  7.3 },
    {    0, 0, 13.0 },
    {    1, 0,    2 },
    { 12.6, 0,    0 },
    {    0, 0, -3.6 }
};

RobotController *RobotController;
MyServo  *servos[6];
Kinematic Kinematic(geometry);

void logicalToPhysicalAngles(float angles[6]) {
    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];
}

void setup()
{
    for (size_t i = 0; i < 6; i++) {
        servos[i] = new MyServo(servoConfig[0],
                                servoConfig[1],
                                servoConfig[2],
                                servoConfig[3],
                                servoConfig[4],
                                servoConfig[5]);
    }

    RobotCon = new RobotController(servos,
                                   Kinematic,
                                   logicalAngleLimits,
                                   logicalToPhysicalAngles,
                                   physicalToLogicalAngles)
}

void loop()
{
  
}
