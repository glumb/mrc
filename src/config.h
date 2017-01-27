#include "Arduino.h"
#include "Kinematic.h"
// ---- Servos ---- todo why not pwm pins? 3-6 9-10 20-23
#define pin_robot_servo_0 3
#define pin_robot_servo_1 4
#define pin_robot_servo_2 5
#define pin_robot_servo_3 6
#define pin_robot_servo_4 9
#define pin_robot_servo_5 10

#define pin_additional_servo_6 20
#define pin_additional_servo_7 -1

// v6
// const float servoConfig[6][6] = {
//     { pin_robot_servo_0, 160.00,   700.00, 2380.00,  -90.00,   90.00 },
//     { pin_robot_servo_1, 160.00,  1880.00,  710.00,  -90.00,   45.00 },
//     { pin_robot_servo_2, 160.00,   640.00, 2330.00, -135.00,   45.00 },
//     { pin_robot_servo_3, 160.00,   740.00, 2260.00,  -90.00,   85.00 },
//     { pin_robot_servo_4, 160.00,  2360.00,  730.00,  -15.00,  140.00 },
//     { pin_robot_servo_5, 160.00,   740.00, 2200.00,  -90.00,   60.00 }
// };
// float geometry[5][3] = { { 4.6, 8, 0 }, { 0, 11.6, 0 }, { 1.5, 2, 0 }, { 11, 0, 0 }, { 0, -3, 0 } };
// #define kinematic_coupling(servos, i) {                                        \
//         if ((i) == 1) {                                                        \
//             servos[2]->setOffset(servos[1]->getCurrentAngleExcludingOffset()); \
//         }                                                                      \
// }

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
//             servos[2]->setOffset(servos[1]->getCurrentAngleExcludingOffset()); \
//         }                                                                      \
// }

// 4 axis
const float servoConfig[6][7] = {
        { pin_robot_servo_0,  250,  570.00, 2400.00,  -77.00,  83.00,   0 },
        { pin_robot_servo_1,  250, 1190.00, 2400.00,  -90.00,  18.00,   0 },
        { pin_robot_servo_2,  250, 2175.00,  968.00, -110.00,  -9.00, -30 },
        { pin_robot_servo_3,  250, 1500.00, 1500.00,  -90.00,  75.00,   0 },
        { pin_robot_servo_4,  250, 1500.00, 1500.00,  -20.00, 135.00,  30 },
        { pin_robot_servo_5,  250, 2250.00,  750.00,  -57.00, 90.00, 0 }
};

float geometry[5][3] = { { 2.5 + 2.3, 7.3, 0 }, { 0, 13.0, 0 }, { 1, 0, 0 }, { 12.6, 0, 0 }, { 0, -3.6, 0 } };

Kinematic::ROBOT_TYPE robotType = Kinematic::ROBOT_TYPE::AXIS4;

#define kinematic_coupling(servos, i) {                                        \
        if ((i) == 1) {                                                        \
            servos[2]->setOffset(servos[1]->getCurrentAngleExcludingOffset()); \
        }                                                                      \
}

const float additionalAxisServoConfig[2][6] = {
    { pin_additional_servo_6, 160.00, 1888.00,    1222,    0.00,  1.00 },
    { pin_additional_servo_7, 160.00,    1000, 2000.00,  -90.00, 90.00 }
};
