#include "config.h"
#include <Arduino.h>
#include "handleSerialCommand.h"
#include "MyServo.h"
#include "TimerOne.h"
#include "Kinematic.h"
#include "RobotController.h"
#include "Logger.h"
#include "Display.h"
#include "IOLogic.h"
#include "AdditionalAxisController.h"

// ---- I2C do not change! ----
#define pin_sda   18
#define pin_scl  19 // SCK
#define pin_latch  17


void updateServos();

MyServo   *servos[8];
Kinematic *Kin;
RobotController *RoboCon;
Display Display;
IOLogic IOLogic;
AdditionalAxisController *AxisController;
handleSerialCommand *serialCommand;

#define SERVOMIN  200  // usually 1000us
#define SERVOMAX  2800 // usually 2000us


Logger mainLogger("main");

// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
// const float servoConfig[6][6] = {
//     { pin_servo_0,     160,    542,   2220,    -90,     90 },
//     { pin_servo_1,     160,   2034,    600,    -90,     62 }, //
//     { pin_servo_2,     160,    548,   2380,   -135,     40 },
//     { pin_servo_3,     160,    760,   2234,    -90,     75 },
//     { pin_servo_4,     160,   2360,    793,    -20,    135 },
//     { pin_servo_5,     160,   2300,    600,   -360,    360 }
// };



void setup()
{
    Serial.begin(9600);

    // --- show start screen ---
    Display.begin();
    Display.clear();
    Display.displayText(0, 0, "STARTING");
    Display.displayRobotGeometry(geometry);
    Display.show();
    delay(500);

    // --- init servos ---

    for (size_t i = 0; i < 6; i++) {
        servos[i] = new MyServo(servoConfig[i][0],
                                servoConfig[i][1] / 180.0 * PI,
                                servoConfig[i][2],
                                servoConfig[i][3],
                                servoConfig[i][4] / 180 * PI,
                                servoConfig[i][5] / 180 * PI,
                                servoConfig[i][6] / 180 * PI
                                );

        // servos[i]->setTargetRadAngle(0);
    }
    // additional axis
    for (size_t i = 0; i < 2; i++) {
        servos[i + 6] = new MyServo(additionalAxisServoConfig[i][0],
                                    additionalAxisServoConfig[i][1] / 180.0 * PI,
                                    additionalAxisServoConfig[i][2],
                                    additionalAxisServoConfig[i][3],
                                    additionalAxisServoConfig[i][4] / 180 * PI,
                                    additionalAxisServoConfig[i][5] / 180 * PI
                                    );
    }

    Kin = new Kinematic(geometry);

    RoboCon = new RobotController(servos, Kin);

    AxisController = new AdditionalAxisController(servos + 6);

    serialCommand = new handleSerialCommand(RoboCon, IOLogic, AxisController);

    // RoboCon->setTargetPose(18, 18, -10, PI, 0, 0);
    RoboCon->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);

    RoboCon->setMaxVelocity(10);

    // RoboCon->setTargetAngle(0,-30.0/180.0*PI);
    // RoboCon->setTargetAngle(1,-10.0/180.0*PI);
    // RoboCon->setTargetAngle(2,-10.0/180.0*PI);

    Timer1.initialize(20 * 1000); // 20ms

    Timer1.attachInterrupt(updateServos);
}

void updateServos() {
    for (size_t i = 0; i < 8; i++) {
        servos[i]->process(20);

        kinematic_coupling(servos, i);
    }

    // important: move to loop when debugging (Serial.print needs ISR)
    // todo use volatile and ATOMIC on angle buffer and stuff
    RoboCon->process();
}

void renderDisplay() {
    Display.clear();
    Display.displayRobot(Kin, servos, 10, 20, 0.5, 1);
    Display.displayRobot(Kin, servos, 35, 28, 0.5, 0);


    // Display.displayBars(64, 8, 64, 6 * 4, servos);
    String firstLine;

    switch (RoboCon->getMovementMethod()) {
    case RobotController::MOVEMENT_METHODS::P2P:
        firstLine += "M00";
        break;

    case RobotController::MOVEMENT_METHODS::LINEAR:
        firstLine += "M01";
        break;

    case RobotController::MOVEMENT_METHODS::CIRCULAR:
        firstLine += "M02";
        break;
    }
    firstLine += " V " + String(RoboCon->getMaxVelocity());
    Display.displayText(0,      0, firstLine);
    Display.displayText(13 * 6, 0, "B " + String(serialCommand->getFullBufferSize()) + "/" + String(serialCommand->getBufferSize()));
    Display.displayBars(64, 8, 64, 6 * 3, servos);

    if (IOLogic.isDone()) {
        Display.displayText(0, 8, "done");
    } else {
        Display.displayText(0, 8, "n done");
    }

    // --- show IO ---
    if (true || !IOLogic.isDone()) {
        unsigned int x = 70;
        unsigned int y = 6 * 3 + 10;

        // display target state
        for (size_t i = 0; i < 10; i++) {
            switch (IOLogic.getTargetState(i)) {
            case IOLogic::IO_LOW:
                display.drawRect(x + 6 * i, y, 4, 3, WHITE);
                break;

            case IOLogic::IO_HIGH:
                display.fillRect(x + 6 * i, y, 4, 3, WHITE);
                break;

            case IOLogic::IO_UNDEFINED:
                display.drawRect(x + 6 * i, y, 4, 1, WHITE);
                break;
            }
        }
    }
    Display.show(); // takes 40 ms!
}

void loop()
{
    static unsigned int displayCounter = 0;

    mainLogger.resetTime();

    // mainLogger.time("before process");
    //
    // mainLogger.time("after process");

    if (displayCounter++ >= 10000) {
        renderDisplay();
        displayCounter = 0;
    }

    // mainLogger.time("after Display");

    serialCommand->listen();
}
