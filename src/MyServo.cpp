#include "MyServo.h"
#include <util/atomic.h>
#include "Logger.h"

namespace {
Logger logger("MyServo");
}


/**
 *
 */
MyServo::MyServo(int          pinNumber,
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

float MyServo::getHomeRadAngle() {
    return this->homeAngle;
}

void MyServo::setAngleLimits(float minRadAngle, float maxRadAngle) {
    if (minRadAngle > maxRadAngle) {
        logger.error("minAngle must be smaller than maxAngle on servo_num: " + String(this->pinNumber) + " (min: " + String(
                         minRadAngle / PI * 180) + " max: " + String(maxRadAngle / PI * 180) + ")");
    }
    this->minRadAngle = minRadAngle;
    this->maxRadAngle = maxRadAngle;
}

float MyServo::getMinRadAngle() {
    return this->minRadAngle;
}

float MyServo::getMaxRadAngle() {
    return this->maxRadAngle;
}

void MyServo::setCalibrationFreq(unsigned int minFreq, unsigned int maxFreq) {
    this->minFreq = minFreq;
    this->maxFreq = maxFreq;
}

unsigned int MyServo::getMinFreq() {
    return this->minFreq;
}

unsigned int MyServo::getMaxFreq() {
    return this->maxFreq;
}

float MyServo::getCurrentAngle() { // physical angle
    float result;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = this->currentAngle;
    }
    return result;
}

void MyServo::setTargetRadAngle(float angleRad) {
    this->startAngle  = this->currentAngle;
    this->elapsedTime = 0;
    this->targetAngle = angleRad;
}

void MyServo::setFreqency(long microseconds) {
    if (!this->virtualServo) this->servo.writeMicroseconds(microseconds);
}

float MyServo::getTargetRadAngle() {
    return this->targetAngle;
}

void MyServo::setCurrentAngleVelocity(float angleRadVelocity) {
    if (angleRadVelocity > this->maxAngleVelocity) {
        this->currentAngleVelocity = this->maxAngleVelocity;
    } else if (angleRadVelocity <= 0) {
        this->currentAngleVelocity = 0.1;
    } else {
        this->currentAngleVelocity = angleRadVelocity;
    }
}

float MyServo::getCurrentAngleVelocity() {
    return this->currentAngleVelocity;
}

float MyServo::getMaxAngleVelocity() {
    return this->maxAngleVelocity;
}

void MyServo::process(unsigned int deltaT) {
    // v = s/t
    this->elapsedTime += deltaT;


    float deltaAngle = this->currentAngleVelocity * this->elapsedTime / 1000.0;

    // Serial.println(deltaAngle);

    if (abs(deltaAngle) > abs(this->targetAngle - this->startAngle)) {
        this->currentAngle = this->targetAngle;

        // set start to target to not move again on elapsed time overflow ~50 days?
        this->startAngle = this->targetAngle;
    } else {
        if (this->targetAngle > this->startAngle) {
            this->currentAngle = this->startAngle + deltaAngle;
        } else {
            this->currentAngle = this->startAngle - deltaAngle;
        }
    }

    // if (abs(deltaAngle) > this->currentAngleVelocity * deltaTSeconds) {
    //     if (deltaAngle > 0) {
    //         this->currentAngle = this->currentAngle + this->currentAngleVelocity * deltaTSeconds;
    //     } else {
    //         this->currentAngle = this->currentAngle - this->currentAngleVelocity * deltaTSeconds;
    //     }
    // } else {
    //     this->currentAngle = this->targetAngle;
    // }

    this->move();

    // }
}

bool MyServo::atTargetAngle() {
    return abs(this->currentAngle - this->targetAngle) < 0.000001;
}

void MyServo::move() {
    // val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    // Serial.print("angle ");
    // logger.info(this->currentAngle/PI*180.0);

    // add offset angle here, to move it at full speed, since its only used for kinemtic coupling of servos
    unsigned int freq = int(
        this->map_float(this->currentAngle, this->minRadAngle, this->maxRadAngle, this->minFreq, this->maxFreq)
        );

    if (!this->virtualServo) this->servo.writeMicroseconds(freq);
}

float MyServo::map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool MyServo::getOutOfRange() {
    return this->outOfRange;
}
