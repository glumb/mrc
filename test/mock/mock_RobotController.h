#ifndef MOCK_ROBOT_CONTROLLER_H
#define MOCK_ROBOT_CONTROLLER_H 1

#include "gmock/gmock.h"
#include "MyServo.h"
#include "Kinematic.h"
#include "RobotController.h"


class mock_RobotController : public RobotController {
public:

    mock_RobotController(MyServo   *servos[],
                         Kinematic& Kin,
                         float      logicalAngleLimits[6][2],
                         void(*_logicalToPhysicalAngles)(float[6]),
                         void(*_physicalToLogicalAngles)(float[6]))
        : RobotController(servos, Kin, logicalAngleLimits, _logicalToPhysicalAngles, _physicalToLogicalAngles) {}

    MOCK_METHOD2(setTargetPose, void(RobotController::POSITION pos, float value));
    MOCK_METHOD1(getCurrentPose, void(float pose[6]));
    MOCK_METHOD1(getCurrentPose, float(RobotController::POSITION pos));
    MOCK_METHOD1(setMaxVelocity, void(float velocity));
    MOCK_METHOD0(getMaxVelocity, float());
    MOCK_METHOD1(setMovementMethod, void(RobotController::MOVEMENT_METHODS method));
    MOCK_METHOD0(getMovementMethod, RobotController::MOVEMENT_METHODS());
    MOCK_METHOD1(getCurrentLogicalAngle,  float(unsigned int index));
    MOCK_METHOD1(getCurrentLogicalAngles, void(float angles[6]));
    MOCK_METHOD2(setTargetLogicalAngle, void(unsigned int index, float angle));
};

#endif // ifndef ROBOT_CONTROLLER_H
