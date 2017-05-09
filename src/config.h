#include "Arduino.h"
#include "Kinematic.h"

// ---- Servos ---- todo why not pwm pins? 3-6 9-10 20-21
#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
#define pin_servo_4 9
#define pin_servo_5 10

#define pin_additional_servo_6 20
#define pin_additional_servo_7 -1

// v6
// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
const float servoConfig[6][7] = {
    { pin_servo_0,  150 * DEG_TO_RAD,  700.00, 2380.00,  -90.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_1,  150 * DEG_TO_RAD,  710.00, 1909.00,  -45.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_2,  150 * DEG_TO_RAD, 2290.00,  650.00,  -45.00 * DEG_TO_RAD, 135.00 * DEG_TO_RAD, 0 },
    { pin_servo_3,  150 * DEG_TO_RAD,  740.00, 2260.00,  -90.00 * DEG_TO_RAD,  85.00 * DEG_TO_RAD, 0 },
    { pin_servo_4,  150 * DEG_TO_RAD,  730.00, 2340.00, -140.00 * DEG_TO_RAD,  15.00 * DEG_TO_RAD, 0 },
    { pin_servo_5,  150 * DEG_TO_RAD,  740.00, 2200.00,  -90.00 * DEG_TO_RAD,  60.00 * DEG_TO_RAD, 0 }
};

// float geometry[5][3] = { { 4.6, 8, 0 }, { 0, 11.6, 0 }, { 1.5, 2, 0 }, { 11, 0, 0 }, { 0, -3, 0 } };
// float geometry[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
float geometry[5][3] = {
    {    5, 0,  7.3 },
    {    0, 0, 13.0 },
    {    1, 0,    2 },
    { 12.6, 0,    0 },
    {    0, 0, -3.6 }
};

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


void logicalToPhysicalAngles(float angles[6]) {
    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];
}

// v7
// const float servoConfig[6][6] = {
//     { pin_robot_servo_0,  150,  549.00, 2089.00, -90.00,  90.00 },
//     { pin_robot_servo_1,  150, 1691.00,  560.00, -89.00,  40.00 },
//     { pin_robot_servo_2,  150, 1310.00, 2373.00, -90.00,  15.00 },
//     { pin_robot_servo_3,  150,  571.00, 2022.00, -75.00,  90.00 },
//     { pin_robot_servo_4,  150,  559.00, 2090.00, -32.00, 150.00 },
//     { pin_robot_servo_5,  150,  722.00, 2231.00, -90.00,  90.00 }
// };
// #define kinematic_coupling(servos, i) {                                        \
//         if ((i) == 1) {                                                        \
//             servos[2]->setOffset(servos[1]->getCurrentAngle()); \
//         }                                                                      \
// }

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

// float logicAngleLimits[6][2] = {
//     {
//         servoConfig[0][4],
//         servoConfig[0][5],
//     },
//     {
//         servoConfig[1][4],
//         servoConfig[1][5],
//     },
//     {
//         -50,
//         90,
//     },
//     {
//         servoConfig[3][4],
//         servoConfig[3][5],
//     },
//     {
//         servoConfig[4][4],
//         servoConfig[4][5],
//     },
//     {
//         servoConfig[5][4],
//         servoConfig[5][5],
//     }
// };


// void logicToPhysicalAngles(float angles[6]) {
//     angles[2] += angles[1];
// }

const float additionalAxisServoConfig[2][7] = {
    { pin_additional_servo_6, 160.00 * DEG_TO_RAD, 1888.00,    1222,    0.00 * DEG_TO_RAD,  1.00 * DEG_TO_RAD, 0 },
    { pin_additional_servo_7, 160.00 * DEG_TO_RAD,    1000, 2000.00,  -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0 }
};
