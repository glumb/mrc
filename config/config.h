#include "Arduino.h"
#include "Kinematic.h"

// ---- Servos ---- on Teensy PWM pins are 3-6 9-10 20-21
#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
#define pin_servo_4 9
#define pin_servo_5 10

#define pin_additional_servo_6 20
#define pin_additional_servo_7 -1

// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
const float servoConfig[6][7] = {
    { pin_servo_0,  300 * DEG_TO_RAD,  700.00, 2380.00,  -90.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_1,  300 * DEG_TO_RAD,  710.00, 1909.00,  -45.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_2,  300 * DEG_TO_RAD, 2290.00,  650.00,  -45.00 * DEG_TO_RAD, 135.00 * DEG_TO_RAD, 0 },
    { pin_servo_3,  300 * DEG_TO_RAD,  740.00, 2260.00,  -90.00 * DEG_TO_RAD,  85.00 * DEG_TO_RAD, 0 },
    { pin_servo_4,  300 * DEG_TO_RAD,  730.00, 2340.00, -140.00 * DEG_TO_RAD,  15.00 * DEG_TO_RAD, 0 },
    { pin_servo_5,  300 * DEG_TO_RAD,  740.00, 2200.00,  -90.00 * DEG_TO_RAD,  60.00 * DEG_TO_RAD, 0 }
};

// mor mp-robot-a/mp-robot-kit
float geometry[5][3] = {
    {    4.6, 0,    7.9 },
    {      0, 0,   11.7 },
    {      1, 0,    1.5 },
    {  12.15, 0,      0 },
    {      0, 0,     -3 }
};

// E.g. joint 0 cant be < 90° to not crash into itself
float logicAngleLimits[6][2] = {
    { servoConfig[0][4],
      servoConfig[0][5] },
    { servoConfig[1][4],
      servoConfig[1][5] },
    { servoConfig[2][4],
      servoConfig[2][5] },
    { servoConfig[3][4],
      servoConfig[3][5] },
    { servoConfig[4][4],
      servoConfig[4][5] },
    { servoConfig[5][4],
      servoConfig[5][5] }
};

// relation between physical and logical angles based on robot kinematic coupling.
void logicalToPhysicalAngles(float angles[6]) {
    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];
}

// 4 axis
// const float servoConfig[6][7] = {
//     { pin_robot_servo_0,  250,  570.00, 2400.00,  -77.00,  83.00,   0 },
//     { pin_robot_servo_1,  250, 1190.00, 2400.00,  -90.00,  18.00,   0 },
//     { pin_robot_servo_2,  250, 2175.00,  968.00, -110.00,  -9.00, -30 },
//     { pin_robot_servo_3,  250, 1500.00, 1500.00,  -90.00,  75.00,   0 },
//     { pin_robot_servo_4,  250, 1500.00, 1500.00,  -20.00, 135.00,  30 },
//     { pin_robot_servo_5,  250, 2281.00,  712.00,  -75.00,  75.00,   0 }
// };

// float geometry[5][3] = { { 2.5 + 2.3, 7.3, 0 }, { 0, 13.0, 0 }, { 1, 0, 0 }, { 12.6, 0, 0 }, { 0, -3.6, 0 } };

// configure additional axis such as grippers and other devices
// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position || angleDegMin/Max can be any value you
// want to map the min and max frequency to.
// It may be usefull to map a gripper to 0-100 based on the percentage of opening.
const float additionalAxisServoConfig[2][7] = {
    { pin_additional_servo_6, 160.00 * DEG_TO_RAD, 1888.00,    1122,    0.00 * DEG_TO_RAD,  1.00 * DEG_TO_RAD, 0 }, // mapped to 0-1, to
                                                                                                                    // open:1, closed:0
    { pin_additional_servo_7, 160.00 * DEG_TO_RAD,    1000, 2000.00,  -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0 }
};

unsigned int pinMap[10] = { 8, 11, 12, 7, 0, 1, 0, 0, 0, 0 }; // todo
