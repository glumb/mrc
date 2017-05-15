#include <Arduino.h>
#include "../src/VarSpeedServo.h"
#include "TimerOne.h"


#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
#define pin_servo_4 9
#define pin_servo_5 10


// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
const float servoConfig[6][7] = {
    { pin_servo_0,  150 * DEG_TO_RAD,  700.00, 2380.00,  -90.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_1,  150 * DEG_TO_RAD,  710.00, 1909.00,  -45.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { pin_servo_2,  150 * DEG_TO_RAD, 2290.00,  650.00,  -45.00 * DEG_TO_RAD, 135.00 * DEG_TO_RAD, 0 },
    { pin_servo_3,  150 * DEG_TO_RAD,  740.00, 2260.00,  -90.00 * DEG_TO_RAD,  85.00 * DEG_TO_RAD, 0 },
    { pin_servo_4,  150 * DEG_TO_RAD,  730.00, 2340.00, -140.00 * DEG_TO_RAD,  15.00 * DEG_TO_RAD, 0 },
    { pin_servo_5,  150 * DEG_TO_RAD,  740.00, 2200.00,  -90.00 * DEG_TO_RAD,  60.00 * DEG_TO_RAD, 0 }
};

VarSpeedServo *servos[6];


void updateServos() {
    for (size_t i = 0; i < 6; i++) {
        servos[i]->process(20);
    }
}

void setup()
{
    Serial.begin(9600);

    for (size_t i = 0; i < 6; i++) {
        servos[i] = new VarSpeedServo(servoConfig[i][0],
                                servoConfig[i][1],
                                servoConfig[i][2],
                                servoConfig[i][3],
                                servoConfig[i][4],
                                servoConfig[i][5],
                                servoConfig[i][6]);
    }


    // init Timer and register callback
    Timer1.initialize(20 * 1000); // 20ms
    Timer1.attachInterrupt(updateServos);
}

void loop()
{
    for (size_t i = 0; i < 6; i++) {
        servos[i]->setTargetRadAngle(30 * DEG_TO_RAD);

        while (!servos[i]->atTargetAngle()) {}
        delay(500);
        servos[i]->setTargetRadAngle(-30 * DEG_TO_RAD);

        while (!servos[i]->atTargetAngle()) {}
        delay(500);
        servos[i]->setTargetRadAngle(0);

        while (!servos[i]->atTargetAngle()) {}
        delay(2000);
    }
}
