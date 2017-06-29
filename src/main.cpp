// #define EXAMPLES 1 //uncomment to use examples
#ifdef EXAMPLES

// #include "../examples/Servos.h"
// #include "../examples/RobotController.h"
# include "../examples/CalibrateServos.h"
#else // ifdef EXAMPLES

// # include "../config/mp-robot-d.h"
# include "../config/mp-robot-kit.h"
# include <Arduino.h>
# include "VarSpeedServo.h"
# include "TimerOne.h"
# include "Kinematic.h"
# include "Logger.h"
# include "Display.h"
# include "fastRW.h"

# include "MRILParser.h"
# include "RobotController.h"
# include "IOLogic.h"
# include "AdditionalAxisController.h"
# include "WaitController.h"

# include "MRCPParser.h"
# include "EEPromStorage.h"
# include "RingBuffer.h"
# include "MRCPR.h"

# include "SerialIO.h"

// ---- I2C do not change! ----
# define pin_sda   18
# define pin_scl  19 // SCK
# define pin_latch  17

# define pin_internal_led 13
# define pin_servo_update_status_led 2

# define RINGBUFFER_SIZE 300

void updateServos();
void onIncomingData(char c);

VarSpeedServo   *servos[8];
Kinematic *Kin;
Display    Display;

MRILParser *Mrilparser;
RobotController *RoboCon;
IOLogic IOLogic(pinMap);
AdditionalAxisController *AxisController;
WaitController WaitController;

SerialIO Serialio;

MRCPR Mrcpr(Serialio);

MRCPParser   *Mrcpparser;
EEPromStorage Eepromstorage;
RingBuffer    Ringbuffer(RINGBUFFER_SIZE);

# define SERVOMIN  200  // usually 1000us
# define SERVOMAX  2800 // usually 2000us

# define updateServosEveryMs 20

namespace {
Logger logger("main");
}

void setup()
{
    Serial.begin(9600);
// Eepromstorage.clear();
    // --- show start screen ---
    Display.begin();
    Display.clear();
    Display.displayText(0, 0, "STARTING");
    Display.displayRobotGeometry(geometry);
    Display.show();
    delay(1000);

    // --- init servos ---

    for (size_t i = 0; i < 6; i++) {
        servos[i] = new VarSpeedServo(servoConfig[i][0],
                                servoConfig[i][1],
                                servoConfig[i][2],
                                servoConfig[i][3],
                                servoConfig[i][4],
                                servoConfig[i][5],
                                servoConfig[i][6]
                                );

        // servos[i]->setTargetRadAngle(0);
    }

    // additional axis
    for (size_t i = 0; i < 2; i++) {
        servos[i + 6] = new VarSpeedServo(additionalAxisServoConfig[i][0],
                                    additionalAxisServoConfig[i][1],
                                    additionalAxisServoConfig[i][2],
                                    additionalAxisServoConfig[i][3],
                                    additionalAxisServoConfig[i][4],
                                    additionalAxisServoConfig[i][5]
                                    );
    }

    // Kinematic
    Kin = new Kinematic(geometry);

    Display.displayText(0, 8 * 1, "KIN");
    Display.show();
    delay(100);

    // Robot Controller
    RoboCon = new RobotController(servos, *Kin, logicAngleLimits, logicalToPhysicalAngles, physicalToLogicalAngles); // todo make function
                                                                                                                     // optional

    Display.displayText(0, 8 * 2, "Con");
    Display.show();
    delay(100);

    // Additional Axis
    AxisController = new AdditionalAxisController(servos + 6);

    Display.displayText(0, 8 * 3, "Axis");
    Display.show();
    delay(100);

    // MRIL Parser
    Mrilparser = new MRILParser(*RoboCon,
                                IOLogic,
                                *AxisController,
                                WaitController,
                                Mrcpr);
    Display.displayText(40, 8 * 1, "MRIL");
    Display.show();
    delay(100);

    // MRCP Parser
    Mrcpparser = new MRCPParser(Eepromstorage,
                                Ringbuffer,
                                *Mrilparser,
                                Mrcpr);
    Display.displayText(40, 8 * 2, "MRCP");
    Display.show();
    delay(100);

    // link MRCP to incoming data
    Serialio.onData(onIncomingData);


    RoboCon->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
    RoboCon->setMaxVelocity(10);

    // init Timer and register callback
    Timer1.initialize(updateServosEveryMs * 1000); // 20ms
    Timer1.attachInterrupt(updateServos);

    pinMode(pin_internal_led,            OUTPUT);
    pinMode(pin_servo_update_status_led, OUTPUT);
    digitalWrite(pin_internal_led, HIGH);
}

void onIncomingData(char c) {
    // Serial.print(c); // send back received data
    Mrcpparser->parseChar(c);
}

volatile long timer;
volatile long maxTimer;

void updateServos() {
    digitalLow(pin_internal_led);

    // timer = micros();

    for (size_t i = 0; i < 8; i++) {
        servos[i]->process(updateServosEveryMs);
    }

    // todo use volatile and ATOMIC on angle buffer and stuff
    digitalHigh(pin_internal_led);


}

void renderDisplay();


void loop()
{

    static unsigned int displayCounter = 0;

    logger.resetTime();

    RoboCon->process();// should be part of ISR, but then the display is not working propery. I2C also requires an interrupt
    // status led
    // digitalWrite(pin_internal_led, HIGH);
    if (displayCounter++ >= 20000) {
        renderDisplay(); // takes ~50 ms
        displayCounter = 0;
    }

    // may want to put RoboCon->process in an interrupt based timer as well
    // drawing the display takes quite some time

    RoboCon->process();
    Serialio.process();
    Mrilparser->process();
    Mrcpparser->process();

    // digitalWrite(pin_internal_led, LOW);

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
    firstLine += " V " + String(RoboCon->getMaxVelocity(), 1);

    switch (Mrcpparser->getMode()) {
    case MRCPParser::MRCPMODE::EEPROM:
        Display.displayText(12 * 6, 0,
                            "W " + String(Eepromstorage.getMessagePointer()) + "/" + String(Eepromstorage.getNumberOfMessages()));
        break;

    case MRCPParser::MRCPMODE::QUEUE:
        Display.displayText(12 * 6, 0, "B " + String(Ringbuffer.getSize()) + "/" + String(Ringbuffer.getCapacity()));
        break;
    }
    Display.displayText(0, 0, firstLine);
    Display.displayBars(64, 8, 64, 6 * 3, servos);

    // if (IOLogic.isDone()) {
    //     Display.displayText(0, 8, "done");
    // } else {
    //     Display.displayText(0, 8, "n done");
    // }

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

#endif // ifdef EXAMPLES
