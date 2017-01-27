#include "MyServo.h"
#include <util/atomic.h>
#include "Logger.h"

namespace {
Logger logger("MyServo");
}

/**
 * handles angle offset induces by kinematic coupling
 * angle excluding offset represents the logical angle used for calculations
 * angle including offset represents the physical servo angle
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
        result = this->currentAngle + this->offsetAngle;
    }
    return result;
}

float MyServo::getCurrentAngleExcludingOffset() { // logical angle
    float result;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        result = this->currentAngle;
    }
    return result;
}

void MyServo::setTargetRadAngle(float angleRad) {
    /*logger.info("inside servo set angle");
       logger.info(angleRad);
       logger.info("inside servo min angle");
       logger.info(this->minRadAngle);
       logger.info("inside servo max angle");
       logger.info(this->maxRadAngle);
     */

    // account for offsetAngle
    // todo maybe just constrain() here in logic angles and in the move method on the overall (offset+target) target angle

    this->targetAngle = angleRad;
}

void MyServo::setFreqency(long microseconds) {
    if (!this->virtualServo) this->servo.writeMicroseconds(microseconds);
}

void MyServo::setOffset(float angleRad) {
    this->offsetAngle = angleRad;
}

float MyServo::getOffset() {
    return this->offsetAngle;
}

float MyServo::getTargetRadAngle() {
    return this->targetAngle + this->offsetAngle;
}

float MyServo::getTargetRadAngleExcludingOffset() {
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
    // if ((micros() - this->lastUpdate) > deltaT * 1000) {

    // logger.info((micros() - this->lastUpdate));
    // logger.info(this->maxAngleVelocity);

    // this->lastUpdate = micros();

    // todo use lerp between start and target angle to avoid incremental error
    float deltaAngle    = this->targetAngle - this->currentAngle;
    float deltaTSeconds = (float)deltaT / 1000.0;

    if (abs(deltaAngle) > this->currentAngleVelocity * deltaTSeconds) {
        if (deltaAngle > 0) {
            this->currentAngle = this->currentAngle + this->currentAngleVelocity * deltaTSeconds;
        } else {
            this->currentAngle = this->currentAngle - this->currentAngleVelocity * deltaTSeconds;
        }
    } else {
        this->currentAngle = this->targetAngle;
    }

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
        this->map_float(this->currentAngle + this->offsetAngle, this->minRadAngle, this->maxRadAngle, this->minFreq, this->maxFreq)
        );

    this->outOfRange = false;

    if (this->maxFreq > this->minFreq) {
        if ((freq < this->minFreq) || (freq > this->maxFreq)) {
            this->outOfRange = true;
            return;
        }
    } else {
        if ((freq > this->minFreq) || (freq < this->maxFreq)) {
            this->outOfRange = true;
            return;
        }
    }

    if (!this->virtualServo) this->servo.writeMicroseconds(freq);
}

float MyServo::map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool MyServo::getOutOfRange() {
    return this->outOfRange;
}
