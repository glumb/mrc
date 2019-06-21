#pragma once

#include <Arduino.h>
#include "servoports.h"

const float servoConfig[6][7] = {
    { pin_servo_0,  150 * DEG_TO_RAD,    852, 2091,    -90 * DEG_TO_RAD,     90 * DEG_TO_RAD, 0 },
    { pin_servo_1,  150 * DEG_TO_RAD,    710, 1780,    -70 * DEG_TO_RAD,     90 * DEG_TO_RAD, 0 },
    { pin_servo_2,  150 * DEG_TO_RAD,   2070,  600,    -90 * DEG_TO_RAD,    138 * DEG_TO_RAD, 0 },
    { pin_servo_3,  150 * DEG_TO_RAD,    650, 2370,    -90 * DEG_TO_RAD,     75 * DEG_TO_RAD, 0 },
    { pin_servo_4,  150 * DEG_TO_RAD,   2370,  860,   -127 * DEG_TO_RAD,     14 * DEG_TO_RAD, 0 },
    { pin_servo_5,  150 * DEG_TO_RAD,   2290,  570,    -75 * DEG_TO_RAD,     86 * DEG_TO_RAD, 0 }
};