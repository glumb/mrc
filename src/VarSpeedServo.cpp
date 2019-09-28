#include "VarSpeedServo.h"

#include <util/atomic.h>
#include "Logger.h"
#include "Logger.h"
#include "Arduino.h"

namespace {
Logger logger("VarSpeedServo");
}


/**
 *
 */
VarSpeedServo::VarSpeedServo(int          pinNumber,
                 float        maxAngleVelocity,
                 unsigned int minFreq,
                 unsigned int maxFreq,
                 float        minRadAngle,
                 float        maxRadAngle,
                 float        homeRadAngle) {
    this->pinNumber            = pinNumber;
    this->maxAngleVelocity     = maxAngleVelocity; // per s
    this->currentAngleVelocity = maxAngleVelocity;
    this->lastUpdate           = micros();

    this->minRadAngle = minRadAngle;
    this->maxRadAngle = maxRadAngle;

    if (minRadAngle > maxRadAngle) {
        logger.error("minAngle must be smaller than maxAngle on servo " + String(pinNumber) + " initialization (min: " + String(
                         minRadAngle / PI * 180) + " max: " + String(maxRadAngle / PI * 180) + ")");
    }

    if (minRadAngle > maxRadAngle) {
        logger.error("minRadAngle must be smaller than maxRadAngle. Servo pin number: " + String(pinNumber));
    }

    this->startAngle   = homeRadAngle;
    this->currentAngle = homeRadAngle;
    this->targetAngle  = homeRadAngle;
    this->homeAngle    = homeRadAngle;

    this->minFreq = minFreq;
    this->maxFreq = maxFreq;

    logger.info("Servo ");
    logger.info(pinNumber);

    if (this->pinNumber < 0) this->virtualServo = true;

    if (!this->virtualServo) {
        this->servo.attach(pinNumber);
    }

    this->move(); // drive to 0 position according to min max angle
}

float VarSpeedServo::getHomeRadAngle() {
    return this->homeAngle;
}

int VarSpeedServo::getPinNumber() {
    return this->pinNumber;
}

void VarSpeedServo::setAngleLimits(float minRadAngle, float maxRadAngle) {
    if (minRadAngle > maxRadAngle) {
        logger.error("minAngle must be smaller than maxAngle on servo_num: " + String(this->pinNumber) + " (min: " + String(
                         minRadAngle / PI * 180) + " max: " + String(maxRadAngle / PI * 180) + ")");
    }
    this->minRadAngle = minRadAngle;
    this->maxRadAngle = maxRadAngle;
}

float VarSpeedServo::getMinRadAngle() {
    return this->minRadAngle;
}

float VarSpeedServo::getMaxRadAngle() {
    return this->maxRadAngle;
}

void VarSpeedServo::setCalibrationFreq(unsigned int minFreq, unsigned int maxFreq) {
    this->minFreq = minFreq;
    this->maxFreq = maxFreq;
}

unsigned int VarSpeedServo::getMinFreq() {
    return this->minFreq;
}

unsigned int VarSpeedServo::getMaxFreq() {
    return this->maxFreq;
}

float VarSpeedServo::getCurrentAngle() { // physical angle
    float result;

    // std::cout << "getCurrentAngle "<<currentAngle << '\n';

    // current angle may change whe process interrupt kicks in
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = this->currentAngle;
    }
    return result;
}

void VarSpeedServo::setTargetRadAngle(float angleRad) {
    this->startAngle  = this->currentAngle;
    this->elapsedTime = 0;
    this->targetAngle = angleRad;
}

void VarSpeedServo::setFreqency(long microseconds) {
    if (!this->virtualServo) this->servo.writeMicroseconds(microseconds);
}

float VarSpeedServo::getTargetRadAngle() {
    return this->targetAngle;
}

void VarSpeedServo::setCurrentAngleVelocity(float angleRadVelocity) {
    if (angleRadVelocity > this->maxAngleVelocity) {
        this->currentAngleVelocity = this->maxAngleVelocity;
    } else if (angleRadVelocity <= 0) {
        this->currentAngleVelocity = 0.1;
    } else {
        this->currentAngleVelocity = angleRadVelocity;
    }
}

float VarSpeedServo::getCurrentAngleVelocity() {
    return this->currentAngleVelocity;
}

float VarSpeedServo::getMaxAngleVelocity() {
    return this->maxAngleVelocity;
}

unsigned int VarSpeedServo::process(unsigned int deltaT) {
    // v = s/t
    this->elapsedTime += deltaT;


    float deltaAngle = this->currentAngleVelocity * this->elapsedTime / 1000.0;

    // Serial.println(deltaAngle);

    if (fabs(deltaAngle) > fabs(this->targetAngle - this->startAngle)) {
        this->currentAngle = this->targetAngle;

        // set start to target to not move again on elapsed time. overflow ~50 days?
        this->startAngle = this->targetAngle;
    } else {
        if (this->targetAngle > this->startAngle) {
            this->currentAngle = this->startAngle + deltaAngle;
        } else {
            this->currentAngle = this->startAngle - deltaAngle;
        }
    }

    return this->move();

}

bool VarSpeedServo::atTargetAngle() {
    return fabs(this->currentAngle - this->targetAngle) < 0.000001;
}

unsigned int VarSpeedServo::move() {
    unsigned int freq = int(
        this->map_float(this->currentAngle, this->minRadAngle, this->maxRadAngle, this->minFreq, this->maxFreq)
        );

    if (!this->virtualServo) this->servo.writeMicroseconds(freq);

    return freq;
}

float VarSpeedServo::map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool VarSpeedServo::getOutOfRange() {
    return this->outOfRange;
}
