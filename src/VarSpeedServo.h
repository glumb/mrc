#ifndef MY_SERVO_H
#define MY_SERVO_H 1

#ifndef MOCK_VIRTUAL // used for setting methods to virtual in test environment
# define MOCK_VIRTUAL
#endif // ifndef MOCK_VIRTUAL

#include <Arduino.h>
#include <Servo.h>


#define DELTA_T 15 // go at 66hz to best match the servo freq

class VarSpeedServo {
public:

    VarSpeedServo(int          pinNumber,
            float        maxAngleVelocity,
            unsigned int minFreq,
            unsigned int maxFreq,
            float        minRadAngle,
            float        maxRadAngle,
            float        homeRadAngle = 0);

    MOCK_VIRTUAL int          getPinNumber();

    MOCK_VIRTUAL void         setAngleLimits(float minRadAngle,
                                             float maxRadAngle);
    MOCK_VIRTUAL float        getMinRadAngle();
    MOCK_VIRTUAL float        getMaxRadAngle();

    MOCK_VIRTUAL void         setCalibrationFreq(unsigned int minFreq,
                                                 unsigned int maxFreq);

    MOCK_VIRTUAL float        getCurrentAngle();

    MOCK_VIRTUAL void         setTargetRadAngle(float angleRad);
    MOCK_VIRTUAL float        getTargetRadAngle();

    MOCK_VIRTUAL void         setCurrentAngleVelocity(float angleRadVelocity);
    MOCK_VIRTUAL float        getCurrentAngleVelocity();

    MOCK_VIRTUAL unsigned int getMinFreq();
    MOCK_VIRTUAL unsigned int getMaxFreq();

    MOCK_VIRTUAL void         setFreqency(long microseconds);

    MOCK_VIRTUAL float        getMaxAngleVelocity();

    MOCK_VIRTUAL bool         getOutOfRange();

    MOCK_VIRTUAL bool         atTargetAngle();

    MOCK_VIRTUAL unsigned int process(unsigned int deltaT = DELTA_T);

    MOCK_VIRTUAL float        getHomeRadAngle();

private:

    bool virtualServo = false;

    int pinNumber;

    float homeAngle;

    unsigned int minFreq;
    unsigned int maxFreq;

    unsigned long elapsedTime = 0;

    float startAngle = 0;
    volatile float currentAngle;
    float targetAngle = 0;

    unsigned long lastUpdate;

    float minRadAngle;
    float maxRadAngle;

    float currentAngleVelocity;
    float maxAngleVelocity;

    bool outOfRange = false;

    Servo servo;

    unsigned int move();

    static float map_float(float x,
                           float in_min,
                           float in_max,
                           float out_min,
                           float out_max);
};
#endif /* ifndef MY_SERVO_H */
