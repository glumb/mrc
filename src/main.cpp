#include "config.h"
#include <Arduino.h>
#include "MyServo.h"
#include "TimerOne.h"
#include "Kinematic.h"
#include "Logger.h"
#include "Display.h"

#include "MRILParser.h"
#include "RobotController.h"
#include "IOLogic.h"
#include "AdditionalAxisController.h"
#include "WaitController.h"

#include "MRCPParser.h"
#include "EEPromStorage.h"
#include "RingBuffer.h"
#include "MRCPR.h"

#include "SerialIO.h"

// ---- I2C do not change! ----
#define pin_sda   18
#define pin_scl  19 // SCK
#define pin_latch  17

#define RINGBUFFER_SIZE 300

void updateServos();
void onIncomingData(char c);

MyServo   *servos[8];
Kinematic *Kin;
Display    Display;

MRILParser *Mrilparser;
RobotController *RoboCon;
IOLogic IOLogic;
AdditionalAxisController *AxisController;
WaitController WaitController;

SerialIO Serialio;

MRCPR Mrcpr(Serialio);

MRCPParser   *Mrcpparser;
EEPromStorage Eepromstorage;
RingBuffer    Ringbuffer(RINGBUFFER_SIZE);

#define SERVOMIN  200  // usually 1000us
#define SERVOMAX  2800 // usually 2000us

#define updateServosEveryMs 15

namespace {
Logger logger("main");
}

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

    // Kinematic
    Kin = new Kinematic(geometry);

    Display.displayText(0, 8 * 1, "KIN");
    Display.show();
    delay(500);

    // logical angle limits
    for (size_t i = 0; i < 6; i++) {
        logicAngleLimits[i][0] = logicAngleLimits[i][0] / 180 * PI;
        logicAngleLimits[i][1] = logicAngleLimits[i][1] / 180 * PI;
    }

    // Robot Controller
    RoboCon = new RobotController(servos, *Kin, logicAngleLimits, logicalToPhysicalAngles, physicalToLogicalAngles); // todo make function
                                                                                                                     // optional

    Display.displayText(0, 8 * 2, "Con");
    Display.show();
    delay(500);

    // Additional Axis
    AxisController = new AdditionalAxisController(servos + 6);

    Display.displayText(0, 8 * 3, "Axis");
    Display.show();
    delay(500);

    // MRIL Parser
    Mrilparser = new MRILParser(*RoboCon,
                                IOLogic,
                                *AxisController,
                                WaitController,
                                Mrcpr);

    // MRCP Parser
    Mrcpparser = new MRCPParser(Eepromstorage,
                                Ringbuffer,
                                *Mrilparser,
                                Mrcpr);

    // link MRCP to incoming data
    Serialio.onData(onIncomingData);


    RoboCon->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
    RoboCon->setMaxVelocity(10);

    // init Timer and register callback
    Timer1.initialize(updateServosEveryMs * 1000); // 20ms
    Timer1.attachInterrupt(updateServos);
}

void onIncomingData(char c) {
    Mrcpparser->parseChar(c);
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

void renderDisplay();


void loop()
{
    static unsigned int displayCounter = 0;

    logger.resetTime();

    // logger.time("before process");
    //
    // logger.time("after process");

    if (displayCounter++ >= 10000) {
        renderDisplay();
        displayCounter = 0;

        // Serial.println(timer);
    }

    // logger.time("after Display");
    Serialio.process();
    Mrcpparser->process();

    for (size_t i = 0; i < 8; i++) {
        if (servos[i]->getOutOfRange()) {
            logger.warning("out of frequency range. servo i: " + String(i) + " minAngle: "  + String(
                               servos[i]->getMinRadAngle() / PI * 180) + " maxAngle: " + String(
                               servos[i]->getMaxRadAngle() / PI * 180) +  " target angle: " +
                           String(servos[i]->getTargetRadAngle() / PI * 180));
        }
    }
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
    Display.displayText(13 * 6, 0, "B " + String(Ringbuffer.getSize()) + "/" + String(Ringbuffer.getCapacity()));
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
