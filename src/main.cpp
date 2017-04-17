#include <Arduino.h>
#include "SerialIO.h"
#include "WaitController.h"
#include "EEPromStorage.h"
#include "MRCPParser.h"
#include "RingBuffer.h"
#include "MRILParser.h"
#include "MyServo.h"
#include "Kinematic.h"
#include "RobotController.h"
#include "Display.h"
#include "IOLogic.h"
#include "AdditionalAxisController.h"
#include "WaitController.h"

void receiveCallback(char c) {
    IO.transmit(c);
}

void TEST_SERIALIO() {
    Serial.println("SERIALIO Expect:");
    Serial.println("112233");
    IO.onData(receiveCallback);

    IO.transmit('1');
    IO.transmit('1');
    IO.transmit("22");
    char s[2] = { '3', '3' };
    IO.transmit(s);
    Serial.println("");
    IO.process();
}

void TEST_WAITCONTROLLER() {
    Serial.println("WAITCONTROLLER:");
    Serial.println("okok");
    WaitController W;
    W.waitMs(500);

    if (!W.isDone()) {
        Serial.print("ok");
    }
    delay(501);

    if (W.isDone()) {
        Serial.println("ok");
    }
}

void TEST_EEPROMSTORAGE() {
    Serial.println("EEPROMSTORAGE:");
    Serial.println("okokok");
    EEPromStorage E;
    E.clear();
    char m[4] = { '1', '2', '3', '4' };
    E.appendMessage(m, 4);
    char m2[4] = { 'a', 'b', 'c', 'd' };
    E.appendMessage(m2, 4);

    char message[10];
    char length = E.getMessage(0, message);

    if (message[1] == m[1]) {
        Serial.print("ok");
    } else {
        Serial.print(message[1]);
    }

    if (message[3] == m[3]) {
        Serial.print("ok");
    } else {
        Serial.print(message[3]);
    }
    E.getMessage(1, message);

    if (message[3] == m2[3]) {
        Serial.println("ok");
    } else {
        Serial.print(message[3]);
    }
    delay(10);
}

void TEST_RINGBUFFER() {
    Serial.println("RINGBUFFER:");
    Serial.println("okok");
    RingBuffer Rb(10);

    Rb.clear();
     char bytes[3] = { '1', '2', '3' };

    Rb.putBytes(bytes, 3);

    char returnedMessage[10];
    unsigned int length = Rb.getMessage(returnedMessage);

    if (length == 3) {
        Serial.println("ok");
    }

    if (returnedMessage[1] == bytes[1]) {
        Serial.println("ok");
    }
}

void l2f(float servo[]) {}

void TEST_MRILPARSER() {
    Serial.println("MRILPARSER:");
    Serial.println("okokok");

    MyServo *servos[6];
    servos[0] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[1] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[2] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[3] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[4] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[5] = new MyServo(1, 2, 3, 4, 5, 6, 7);

    float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
    Kinematic K(geo);

    RobotController R(servos, &K, l2f);
    IOLogic IO;
    AdditionalAxisController A(servos);
    WaitController  W;
    MRCPR Mrcpr;

    MRILParser MRIL(&R,IO,&A,W,Mrcpr);

    char mri[] = {'W','3','0'};
    MRIL.parse(mri,3);
    if (!W.isDone()) {
        Serial.print("ok");
    }
    delay(350);

    if (W.isDone()) {
        Serial.println("ok");
    }
}
//
// void TEST_MRILPARSER() {
//     Serial.println("MRILPARSER:");
//     Serial.println("okokok");
//
//     MyServo *servos[2];
//     servos[0] = new MyServo(1, 2, 3, 4, 5, 6, 7);
//     servos[1] = new MyServo(1, 2, 3, 4, 5, 6, 7);
//     Kinematic *K;
//     float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
//     K = new Kinematic(geo);
//     IOLogic IO();
//     AdditionalAxisController A(servos);
//     WaitController  W();
//     RobotController R(servos, K, l2f);
//     MRCPR Mrcpr();
//     EEPromStorage E();
//
//     MRILParser MRIL(&R,IO,&A,W,Mrcpr);
//     MRCPParser M(E, R, MRIL);
//
//
//
// }

// void TEST_MRCPPARSER() {
//     Serial.println("MRCPPARSER:");
//     Serial.println("okokok");
//     EEPromStorage E;
//     RingBuffer R(10);
//     MRILParser(RobotController          *rc,
//                IOLogic                   _IOLogic,
//                AdditionalAxisController *AxisCon,
//                WaitController            WaitCon,
//                MRCPR                     _MRCPR)
//     MRCPParser M(E , R , M );
//
//
//     char message[10];
//     char length = E.getMessage(0, message);
//
//     if (message[1] == m[1]) {
//         Serial.print("ok");
//     } else {
//         Serial.print(message[1]);
//     }
//
//     if (message[3] == m[3]) {
//         Serial.print("ok");
//     } else {
//         Serial.print(message[3]);
//     }
//     E.getMessage(1, message);
//
//     if (message[4] == m2[4]) {
//         Serial.println("ok");
//     } else {
//         Serial.print(message[4]);
//     }
// }

void setup()
{
    delay(4000);

    TEST_SERIALIO();
    delay(1000);
    TEST_WAITCONTROLLER();
    delay(1000);
    TEST_EEPROMSTORAGE();
    delay(1000);
    TEST_MRILPARSER();
    delay(1000);
    TEST_RINGBUFFER();
    delay(1000);

    // TEST_MRCPPARSER();
}

void loop()
{}
