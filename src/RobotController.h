

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
# define ROBOT_CONTROLLER_H

#ifndef MOCK_VIRTUAL // used for setting methods to virtual in test environment
#define MOCK_VIRTUAL
#endif

#include "VarSpeedServo.h"
#include "Kinematic.h"

# define NUMBER_OF_AXIS 6

class RobotController {
public:

    RobotController(VarSpeedServo   *servos[],
        Kinematic &_Kinematic,
        float logicalAngleLimits[6][2],
        void(*_logicalToPhysicalAngles)(float[6]),
      void(*_physicalToLogicalAngles)(float[6]));

    enum MOVEMENT_METHODS { LINEAR=1, P2P=0, CIRCULAR=2 };
    enum STATES { IDLE, MOVING, START_MOVE, PREPARE_MOVE };
    enum POSITION { X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5 };

    // when calling process in a loop, transaction prevents execution of a half set command
    void  startTransaction();
    void  endTransaction();

    void stop();

    MOCK_VIRTUAL void setMaxVelocity(float velocity);
    MOCK_VIRTUAL float getMaxVelocity();

    MOCK_VIRTUAL void setMovementMethod(MOVEMENT_METHODS method);
    MOCK_VIRTUAL MOVEMENT_METHODS getMovementMethod();

    MOCK_VIRTUAL void  getCurrentPose(float currentPose[6]);
    MOCK_VIRTUAL float getCurrentPose(POSITION position);

    MOCK_VIRTUAL void setTargetPose(float x,
           float y,
           float z,
           float a,
           float b,
           float c);
    MOCK_VIRTUAL void setTargetPose(float pose[6]);
      MOCK_VIRTUAL void  setTargetPose(POSITION position,
            float    value);
         MOCK_VIRTUAL   void getTargetPose(float targetPose[6]);
         MOCK_VIRTUAL   float getTargetPose(POSITION position);


            // Logical
    MOCK_VIRTUAL void  getCurrentLogicalAngles(float currentAngles[6]);
    MOCK_VIRTUAL float  getCurrentLogicalAngle(unsigned int index);

    MOCK_VIRTUAL void  setTargetLogicalAngles(float targetAngles[6]); //logical
    MOCK_VIRTUAL void  setTargetLogicalAngle(unsigned int index,
             float        targetAngle);
    MOCK_VIRTUAL void  getTargetLogicalAngles(float targetAngles[6]);
    MOCK_VIRTUAL float getTargetLogicalAngle(unsigned int index);

      // Physical
          MOCK_VIRTUAL void  getCurrentPhysicalAngles(float angles[6]);
          MOCK_VIRTUAL float  getCurrentPhysicalAngle(unsigned int index);

    // void  setTargetPhysicalAngles(float targetAngles[6]); //logical
    // MOCK_VIRTUAL void  setTargetPhysicalAngle(unsigned int index,
            //  float        targetAngle);
    MOCK_VIRTUAL void  getTargetPhysicalAngles(float targetAngles[6]);
    MOCK_VIRTUAL float getTargetPhysicalAngle(unsigned int index);

    MOCK_VIRTUAL bool  isMoving();

    void  process();
private:
  float startAngles[6]     = { 0 };
  float targetAngles[6]     = { 0 };
  float targetPose[6]      = { 0 };

  float logicalAngleLimits[6][2];

  bool targetAnglesChanged = false;
  bool targetPoseChanged = false;


    float interpolationRotationAngle     = 0.0;
    float rotationAxisVectorNorm[3]      = { 0 };
    float targetOrientationVectorNorm[3] = { 0 };
    float targetOrientationVector[3]     = { 0 };
    float startOrientationVectorNorm[3]  = { 0 };
    float startOrientationVector[3]      = { 0 };
    float startPose[6]       = { 0 };

    bool moveAsFarAsPossibleOnOutOfBound;
    bool inTransaction = false;
    STATES state;

    float interpolationDistanceIncrement;
    float interpolationOrientationAngleIncrement;
    float maxVelocity;


    Kinematic &_Kinematic;
    VarSpeedServo *Servos[6];
    void (*logicalToPhysicalAngles )(float[6]);
    void (*physicalToLogicalAngles )(float[6]);


    MOVEMENT_METHODS movementMethod = MOVEMENT_METHODS::P2P;

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

       bool  _poseEquals(float Pose0[6],
            float Pose1[6]);
            bool _anglesEqual(float angles1[6], float angles2[6]);
};

#endif // ifndef ROBOT_CONTROLLER_H
