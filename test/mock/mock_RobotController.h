#ifndef MOCK_ROBOT_CONTROLLER_H
#define MOCK_ROBOT_CONTROLLER_H 1

#include "gmock/gmock.h"
#include "RobotController.h"



class mock_RobotController : public RobotController {
public:

    mock_RobotController(MyServo   *servos[],
                         Kinematic *Kin,
                         void(*logicToPhysicalAngles)(float[6]))
        : RobotController(servos,Kin,logicToPhysicalAngles){};

    MOCK_METHOD2(setTargetPose, void(RobotController::POSITION pos, float value));
};

#endif // ifndef ROBOT_CONTROLLER_H
