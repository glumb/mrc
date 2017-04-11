#include "IOLogic.h"
#include "fastRW.h"
#include "Logger.h"
#include "Arduino.h"

namespace {
Logger logger("IOLogic");
}

IOLogic::IOLogic() {}


void IOLogic::addCondition(unsigned int pin, unsigned int state) {
    pinAsInput(pin);
    logger.info("setting pin as input " + String(pin));

    this->conditionBuffer[this->conditionBufferLength][0] = pin;
    this->conditionBuffer[this->conditionBufferLength][1] = state;
    this->conditionBufferLength++;
}

void IOLogic::setOutput(unsigned int pin, unsigned int state) {
    pinAsOutput(pin);

    Serial.print("setting pin ");
    Serial.print(pin);
    Serial.print("State");
    logger.info(state);

    switch (state) {
    case IOLogic::IO_HIGH:

        digitalHigh(pin);
        break;

    case IOLogic::IO_LOW:
        digitalLow(pin);
        break;

    default:
        Serial.print("expected state [0,1], given: ");
        logger.info(state);
    }
}

bool IOLogic::isDone()                                           {
    if (this->conditionBufferLength == 0) {
        return true;
    } else {
        for (unsigned char i = 0; i < this->conditionBufferLength; i++) {
            unsigned int pin   = this->conditionBuffer[i][0];
            unsigned int state = this->conditionBuffer[i][1];

            /*
                  Serial.print("waitung for pin - state ");
                  Serial.print(pin);
                  Serial.print(" - ");
                  logger.info(state);
                  delay(2000);
             */
            switch (state) {
            case IOLogic::IO_HIGH:

                if (isLow(pin)) {
                    return false;
                }
                break;

            case IOLogic::IO_LOW:

                if (isHigh(pin)) {
                    return false;
                }
                break;

            default:
                Serial.print("expected state [0,1], given: ");
                logger.info(state);
            }
        }

        // all done, reset the buffer
        this->conditionBufferLength = 0;
        return true;
    }
}

unsigned int IOLogic::getTargetState(unsigned int pin) {
    for (size_t i = 0; i < this->conditionBufferLength; i++) {
        if (this->conditionBuffer[i][0] == pin) {
            return this->conditionBuffer[i][1];
        }
    }
    return 2;
}
