#include <Arduino.h>
#include "../src/VarSpeedServo.h"
#include "TimerOne.h"

// use w: up, s: down, d: print, q: goto 0 angle, e,r,t,z:set min_max_angle_freq

// ---- Servos ---- Teensy 3.2 PWM pins 3-6 9-10 20-23
#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
#define pin_servo_4 9
#define pin_servo_5 10
#define pin_servo_6 20
#define pin_servo_7 21


enum mode {
    CALIBRATE,
    TEST
};

mode MODE = CALIBRATE;


void printHelp();

VarSpeedServo *servos[6];

#define SERVOMIN  200  // usually 1000us
#define SERVOMAX  2800 // usually 2000us


// pinNumber, current fre, calibMin frequency, calibMax frequency, angleDegMin, angleDegMax, home position
float tmpServoConfig[6][6] = {
    { pin_servo_0,     1500,    1500,    1500,    -90 * DEG_TO_RAD,     90 * DEG_TO_RAD },
    { pin_servo_1,     1500,    1500,    1500,    -90 * DEG_TO_RAD,     62 * DEG_TO_RAD }, //
    { pin_servo_2,     1500,    1500,    1500,   -135 * DEG_TO_RAD,     40 * DEG_TO_RAD },
    { pin_servo_3,     1500,    1500,    1500,    -90 * DEG_TO_RAD,     75 * DEG_TO_RAD },
    { pin_servo_4,     1500,    1500,    1500,    -20 * DEG_TO_RAD,    135 * DEG_TO_RAD },
    { pin_servo_5,     1500,    1500,    1500,   -180 * DEG_TO_RAD,    180 * DEG_TO_RAD }
};

const float servoConfig[6][7] = {
    { pin_servo_0,  150 * DEG_TO_RAD,    852, 2091,    -90 * DEG_TO_RAD,     90 * DEG_TO_RAD, 0 },
    { pin_servo_1,  150 * DEG_TO_RAD,    710, 1780,    -70 * DEG_TO_RAD,     90 * DEG_TO_RAD, 0 },
    { pin_servo_2,  150 * DEG_TO_RAD,   2070,  600,    -90 * DEG_TO_RAD,    138 * DEG_TO_RAD, 0 },
    { pin_servo_3,  150 * DEG_TO_RAD,    650, 2370,    -90 * DEG_TO_RAD,     75 * DEG_TO_RAD, 0 },
    { pin_servo_4,  150 * DEG_TO_RAD,   2370,  860,   -127 * DEG_TO_RAD,     14 * DEG_TO_RAD, 0 },
    { pin_servo_5,  150 * DEG_TO_RAD,   2290,  570,    -75 * DEG_TO_RAD,     86 * DEG_TO_RAD, 0 }
};


void setup()
{

    // --- init servos ---
    Serial.begin(115200);

    for (size_t i = 0; i < 5; i++) {
        Serial.println("Starting...");
        delay(1000);
    }

    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 6; j++) {
            tmpServoConfig[i][j] = servoConfig[i][j];
        }
        tmpServoConfig[i][1] = 1500; // we dont need velocity, but current frequency
    }

    printHelp();

    for (size_t i = 0; i < 6; i++) {
        servos[i] = new VarSpeedServo(tmpServoConfig[i][0],
                                100, // velocity
                                tmpServoConfig[i][2],
                                tmpServoConfig[i][3],
                                tmpServoConfig[i][4],
                                tmpServoConfig[i][5],
                                0);

        // set the servos to their middle position
        if (MODE == CALIBRATE) servos[i]->setFreqency(1500);
    }
}

unsigned int targetAnglePointer = 0;

unsigned int selectedServo = 0;
unsigned int frequency     = (SERVOMAX + SERVOMIN) / 2;
unsigned int increment     = 1;

unsigned int count = 5;

enum SETMODE { FREQ_MIN, FREQ_MAX, ANGLE_MIN, ANGLE_MAX, MOVE };
SETMODE setmode = MOVE;

float currentFrequency[6] = { 1500, 1500, 1500, 1500, 1500, 1500 };
float currentAngles[6] = { 0 };

#define INCREMENT 'w'
#define DECREMENT 's'
#define CHANGE_MIN_ANGLE 'e'
#define CHANGE_MAX_ANGLE 'r'
#define CHANGE_MIN_FREQUENCY 't'
#define CHANGE_MAX_FREQUENCY 'z'
#define MOVE_TO_ZERO 'q'
#define SET_MOVE 'n'
#define PRINT_CONFIG 'p'
#define CHANGE_MODE 'm'
#define PRINT_HELP 'h'

void printHelp() {
    Serial.println("---------------------------------");
    delay(10);
    Serial.println("Press the listed keys to activate the respective function.");
    delay(10);
    Serial.println("Changing the values autamatically safes them.");
    delay(10);
    Serial.println("");
    delay(10);
    Serial.println("select the servo using numbers [0-5]");
    delay(10);
    Serial.println("increment value: " + String(INCREMENT));
    delay(10);
    Serial.println("decrement value: " + String(DECREMENT));
    delay(10);
    Serial.println("change freq without setting: " + String(SET_MOVE));
    delay(10);
    Serial.println("change min angle: " + String(CHANGE_MIN_ANGLE));
    delay(10);
    Serial.println("change max angle: " + String(CHANGE_MAX_ANGLE));
    delay(10);
    Serial.println("change min frequency: " + String(CHANGE_MIN_FREQUENCY));
    delay(10);
    Serial.println("change max frequency: " + String(CHANGE_MAX_FREQUENCY));
    delay(10);
    Serial.println("move servo to angle 0 degree: " + String(MOVE_TO_ZERO) + " (used to check proper home position)");
    delay(10);
    Serial.println("print config: " + String(PRINT_CONFIG));
    delay(10);


    Serial.println("---------------------------------");
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
    byte serialInput = 0;
    bool changed     = false;

    if (Serial.available() > 0) {
        // read the incoming byte:
        serialInput = Serial.read();

        switch (serialInput) {
        case PRINT_HELP:
            printHelp();
            break;

        case CHANGE_MIN_ANGLE:
            setmode = ANGLE_MIN;
            changed = true;
            break;

        case CHANGE_MAX_ANGLE:
            setmode = ANGLE_MAX;
            changed = true;
            break;

        case CHANGE_MIN_FREQUENCY:

            if (setmode == FREQ_MIN) { // set current angle to frequency on double press key
                tmpServoConfig[selectedServo][1] = tmpServoConfig[selectedServo][2];
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
            }
            setmode = FREQ_MIN;
            changed = true;
            break;

        case CHANGE_MAX_FREQUENCY:

            if (setmode == FREQ_MAX) {
                tmpServoConfig[selectedServo][1] = tmpServoConfig[selectedServo][3];
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
            }
            setmode = FREQ_MAX;
            changed = true;
            break;

        case SET_MOVE:
            setmode = MOVE;
            changed = true;
            break;

        case '0':
            selectedServo = 0;
            changed       = true;
            break;

        case '1':
            selectedServo = 1;
            changed       = true;
            break;

        case '2':
            selectedServo = 2;
            changed       = true;
            break;

        case '3':
            selectedServo = 3;
            changed       = true;
            break;

        case '4':
            selectedServo = 4;
            changed       = true;
            break;

        case '5':
            selectedServo = 5;
            changed       = true;
            break;

        case MOVE_TO_ZERO:
            Serial.println("moving to angle: 0 degree ");
            servos[selectedServo]->setTargetRadAngle(0);
            servos[selectedServo]->process(10000); // high number to fake high interval and move to target position

            changed = true;
            break;


        case PRINT_CONFIG:

            Serial.println("{ ");

            for (size_t i = 0; i < 6; i++) {
                Serial.print("{ pin_servo_" + String(i) + ", ");
                Serial.print(" 150*DEG_TO_RAD, ");

                for (size_t j = 2; j < 6; j++) {
                    if ((j == 1) || (j == 4) || (j == 5) || (j == 6)) {
                        Serial.print(tmpServoConfig[i][j]);
                        Serial.print("*DEG_TO_RAD");
                    } else {
                        Serial.print(tmpServoConfig[i][j]);
                    }
                    delay(10);

                    if (j < 5) Serial.print(", ");
                }

                if (i < 5) {
                    Serial.println(", 0 },");
                } else {
                    Serial.println(", 0 }");
                }
            }
            Serial.println(" }");
            break;

        case CHANGE_MODE:

            if (MODE == CALIBRATE) {
                MODE = TEST;
            } else {
                MODE = CALIBRATE;
            }
        }
    }

    switch (MODE) {
    case CALIBRATE: {
        switch (setmode) {
        case ANGLE_MIN:
        case ANGLE_MAX:

            if (serialInput == INCREMENT) {
                tmpServoConfig[selectedServo][(setmode == ANGLE_MIN) ? 4 : 5]++;
                servos[selectedServo]->setAngleLimits(tmpServoConfig[selectedServo][4],
                                                      tmpServoConfig[selectedServo][5]);
                changed = true;
            } else if (serialInput == DECREMENT) {
                tmpServoConfig[selectedServo][(setmode == ANGLE_MIN) ? 4 : 5]--;
                servos[selectedServo]->setAngleLimits(tmpServoConfig[selectedServo][4],
                                                      tmpServoConfig[selectedServo][5]);
                changed = true;
            }
            break;

        case FREQ_MAX:
        case FREQ_MIN:

            if  (serialInput == INCREMENT) {
                tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3]  = tmpServoConfig[selectedServo][1];
                tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3] += increment;
                tmpServoConfig[selectedServo][1]                              =
                    tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3];
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
                servos[selectedServo]->setCalibrationFreq(tmpServoConfig[selectedServo][2], tmpServoConfig[selectedServo][3]);
                changed = true;
            } else if (serialInput == DECREMENT) {
                tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3]  = tmpServoConfig[selectedServo][1];
                tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3] -= increment;
                tmpServoConfig[selectedServo][1]                              =
                    tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3];
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
                servos[selectedServo]->setCalibrationFreq(tmpServoConfig[selectedServo][2], tmpServoConfig[selectedServo][3]);
                changed = true;
            }
            break;

        case MOVE:

            if (serialInput == INCREMENT) {
                tmpServoConfig[selectedServo][1] += 10;
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
                changed = true;
            } else if (serialInput == DECREMENT) {
                tmpServoConfig[selectedServo][1] -= 10;
                servos[selectedServo]->setFreqency(tmpServoConfig[selectedServo][1]);
                changed = true;
            }
            break;
        }


        if (changed) {
            String mode;

            switch (setmode) {
            case ANGLE_MIN:
                mode = "ANGLE_MIN angle: " + String(tmpServoConfig[selectedServo][(setmode == ANGLE_MIN) ? 4 : 5]);
                break;

            case ANGLE_MAX:
                mode = "ANGLE_MAX angle: " + String(tmpServoConfig[selectedServo][(setmode == ANGLE_MIN) ? 4 : 5]);
                break;

            case FREQ_MIN:
                mode = "FREQ_MIN freq: " + String(tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3]);
                break;

            case FREQ_MAX:
                mode = "FREQ_MAX freq: " + String(tmpServoConfig[selectedServo][(setmode == FREQ_MIN) ? 2 : 3]);
                break;

            case MOVE:
                mode = "MOVE freq: " + String(tmpServoConfig[selectedServo][1]);
                break;
            }
            Serial.println(mode + " servo:" + String(selectedServo) + " angle:" +
                           String(
                               map_float(tmpServoConfig[selectedServo][1], tmpServoConfig[selectedServo][2],
                                               tmpServoConfig[selectedServo][3], tmpServoConfig[selectedServo][4],
                                               tmpServoConfig[selectedServo][5])
                               ));
            frequency = constrain(frequency, SERVOMIN, SERVOMAX);
        }

        // unsigned int val = analogRead(pin_pot); // read the value from the sensor
        delay(50);
        break;
    }

    case TEST:

        // bool allAtTargetAngle = true;
        //
        // for (size_t i = 0; i < 6; i++) {
        //     // Serial.println("check" + String(i));
        //     // Serial.println(servos[i]->getTargetRadAngle());
        //     // Serial.println(servos[i]->getCurrentAngle());
        //     // delay(300);
        //
        //     if (!servos[i]->atTargetAngle()) {
        //         allAtTargetAngle = false;
        //     }
        // }
        //
        // if (allAtTargetAngle) {
        //     targetAnglePointer++;
        //     targetAnglePointer %= targetAngleCount;
        //     Serial.println("At target angle");
        //     Serial.print("pointer ");
        //     Serial.println(targetAnglePointer);
        //
        //     for (size_t r = 0; r < 6; r++) {
        //         servos[r]->setTargetRadAngle(targetAngles[targetAnglePointer][r] / 180 * PI);
        //     }
        // }
        //
        // for (size_t i = 0; i < 6; i++) {
        //     servos[i]->process();
        //
        //     if (i == 1) {
        //         // ---- Kinematic coupling ----
        //         servos[2]->setOffset(servos[1]->getCurrentAngleExcludingOffset());
        //     }
        // }
        break;
    }
}
