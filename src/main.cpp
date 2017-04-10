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
#include "WaitController.h"

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
WaitController WaitController;
AdditionalAxisController *AxisController;
handleSerialCommand *serialCommand;

#define SERVOMIN  200  // usually 1000us
#define SERVOMAX  2800 // usually 2000us

#define updateServosEveryMs 15

Logger mainLogger("main");

void setup()
{
    Serial.begin(9600);

    // --- show start screen ---
    Display.begin();
    Display.clear();
    Display.displayText(0, 0, "STARTING");
    Display.displayRobotGeometry(geometry);
    Display.show();
    delay(2000);

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

    Display.displayText(0, 8 * 1, "KIN");
    Display.show();
    delay(500);

    RoboCon = new RobotController(servos, Kin, logicToPhysicalAngles); // todo make function optional

    for (size_t i = 0; i < 6; i++) {
        logicAngleLimits[i][0] = logicAngleLimits[i][0] / 180 * PI;
        logicAngleLimits[i][1] = logicAngleLimits[i][1] / 180 * PI;
    }
    RoboCon->setLogicAngleLimits(logicAngleLimits);

    Display.displayText(0, 8 * 2, "Con");
    Display.show();
    delay(500);

    AxisController = new AdditionalAxisController(servos + 6);

    Display.displayText(0, 8 * 3, "Axis");
    Display.show();
    delay(500);

    serialCommand = new handleSerialCommand(RoboCon, IOLogic, AxisController, WaitController);

    // RoboCon->setTargetPose(18, 18, -10, PI, 0, 0);
    RoboCon->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);

    RoboCon->setMaxVelocity(10);

    // RoboCon->setTargetAngle(0,-30.0/180.0*PI);
    // RoboCon->setTargetAngle(1,-10.0/180.0*PI);
    // RoboCon->setTargetAngle(2,-10.0/180.0*PI);

    Timer1.initialize(updateServosEveryMs * 1000); // 20ms


    Timer1.attachInterrupt(updateServos);
}

volatile long timer;

void updateServos() {
    timer = micros();

    for (size_t i = 0; i < 8; i++) {
        servos[i]->process(updateServosEveryMs);
    }

    // important: move to loop when debugging (Serial.print needs ISR)
    // todo use volatile and ATOMIC on angle buffer and stuff
    RoboCon->process();

    timer = micros() - timer;
}

void renderDisplay() {
    Display.clear();
    Display.displayRobot(Kin, RoboCon, 10, 20, 0.5, 1);
    Display.displayRobot(Kin, RoboCon, 35, 28, 0.5, 0);


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
        // Serial.println(timer);
    }

    // mainLogger.time("after Display");

    serialCommand->listen();

    for (size_t i = 0; i < 8; i++) {
        if (servos[i]->getOutOfRange()) {
            logger.warning("out of frequency range. servo i: " + String(i) + " minAngle: "  + String(
                               servos[i]->getMinRadAngle() / PI * 180) + " maxAngle: " + String(
                               servos[i]->getMaxRadAngle() / PI * 180) +  " target angle: " +
                           String(servos[i]->getTargetRadAngle() / PI * 180));
        }
    }
}
