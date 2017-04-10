#include "MyServo.h"

#include "Kinematic.h"

/**
 * Robot: geometry, servos, angleLimits, kinematic
 * RobotController accepts target poses and interpolation methods.
 * on sucessive calls to compute() it calculates the target Pose and sets the servo angles accordingly
 *
 * Kinematic must implement forward() and inverse()
 * Servos must implement atTargetAngle(), getTargetRadAngle(), setTargetRadAngle(), setCurrentAngleVelocity(), getMaxAngleVelocity(),
 * getCurrentLogicAngle()
 * todo use interface
 */

#ifndef ROBOT_CONTROLLER_H
# define ROBOT_CONTROLLER_H value


# define NUMBER_OF_AXIS 6

class RobotController {
public:

    RobotController(MyServo   *servos[],
                    Kinematic *Kin,
                    void(*logicToPhysicalAngles)(float[6]));

    enum MOVEMENT_METHODS { LINEAR, P2P, CIRCULAR };
    enum STATES { IDLE, MOVING, START_MOVE, PREPARE_MOVE };
    enum POSITION { X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5 };

    void             setLogicAngleLimits(float angleLimitsRad[6][2]);

    void             setMaxVelocity(float velocity);
    float            getMaxVelocity();

    void             resetPose();
    void             moveToMinPose();
    void             moveToMaxPose();
    void             setMaxAngleVelocity(unsigned int index,
                                         float        angleVelocity);
    void             setMaxAngleVelocities(float angleVelocity);

    void             setMovementMethod(MOVEMENT_METHODS method);
    MOVEMENT_METHODS getMovementMethod();

    void             getTargetPose(float targetPose[6]);
    void             setTargetPose(float x,
                                   float y,
                                   float z,
                                   float a,
                                   float b,
                                   float c);
    void  setTargetPose(POSITION position,
                        float    value);

    void  getCurrentLogicAngles(float currentAngles[6]);
    void  getCurrentLogicAngle(unsigned int index);

    void  getCurrentPhysicalAngles(float angles[6]);

    void  setTargetAngles(float targetAngles[6]);
    void  setTargetAngle(unsigned int index,
                         float        targetAngle);
    void  getTargetAngles(float targetAngles[6]);
    float getTargetAngle(unsigned int index);

    void  getCurrentPose(float currentPose[6]);

    bool  isMoving();
    bool  atTargetPose();
    bool  PoseEquals(float Pose0[6],
                     float Pose1[6]);
    void  process();
    void  startTransaction();
    void  endTransaction();

private:

    float interpolationRotationAngle     = 0.0;
    float rotationAxisVectorNorm[3]      = { 0 };
    float targetOrientationVectorNorm[3] = { 0 };
    float targetOrientationVector[3]     = { 0 };
    float targetPose[6]                  = { 0 };
    float targetPoseBuffer[6]            = { 0 };
    float startOrientationVectorNorm[3]  = { 0 };
    float startOrientationVector[3]      = { 0 };
    float startPose[6]                   = { 0 };
    float startAngles[6]                 = { 0 };
    float targetAngleBuffer[6]           = { 0 }; // needed to be able to set angles once at a time

    bool moveAsFarAsPossibleOnOutOfBound;
    bool inTransaction = false;
    STATES state;

    float interpolationDistanceIncrement;
    float interpolationOrientationAngleIncrement;
    float maxVelocity;

    float logicAngleLimits[6][2];

    void (*logicToPhysicalAngles )(float[6]);

    Kinematic *IK;

    MOVEMENT_METHODS movementMethod;

    MyServo *Servos[6];

    // void (*kinematicCoupling)(float [6]);

    void _setTargetPose(float x,
                        float y,
                        float z,
                        float a,
                        float b,
                        float c);
    void _applyTimedTargetAngles(float targetAngles[6],
                                 float targetTime = 0);

    void rodrigues(float ret[3],
                   float unit[3],
                   float v[3],
                   float angleRad);
};

#endif // ifndef ROBOT_CONTROLLER_H
