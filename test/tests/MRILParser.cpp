#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Arduino.h"

SerialMock *serialMock = serialMockInstance();


#include "../fake/fastRW.h"

#include "MyServo.h"

#include "Kinematic.h"
#include "mock_RobotController.h"

#include "MRCPR.h"
#include "MRILParser.h"

#include "IOLogic.h"

#include "AdditionalAxisController.h"


void l2f(float f[]) {}

TEST(MRILParserTest, setOutputToHigh)
{
    MyServo *servos[6];

    servos[0] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[1] = new MyServo(2, 2, 3, 4, 5, 6, 7);
    servos[2] = new MyServo(3, 2, 3, 4, 5, 6, 7);
    servos[3] = new MyServo(4, 2, 3, 4, 5, 6, 7);
    servos[4] = new MyServo(5, 2, 3, 4, 5, 6, 7);
    servos[5] = new MyServo(6, 2, 3, 4, 5, 6, 7);

    float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
    Kinematic K(geo);

    mock_RobotController R(servos, &K, l2f);
    IOLogic IO;
    AdditionalAxisController A(servos);
    WaitController W;
    MRCPR Mrcpr;

    MRILParser MRILp(R, IO, A, W, Mrcpr);


    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalWrite(8, HIGH));
    EXPECT_CALL(*arduinoMock, pinMode(8, OUTPUT));

    char instruction[3] = { 'O', '8', '1' };

    MRILp.parse(instruction, 3);

    releaseArduinoMock();
}

using ::testing::_;

TEST(MRILParserTest, setX)
{
    MyServo *servos[6];

    servos[0] = new MyServo(1, 2, 3, 4, 5, 6, 7);
    servos[1] = new MyServo(2, 2, 3, 4, 5, 6, 7);
    servos[2] = new MyServo(3, 2, 3, 4, 5, 6, 7);
    servos[3] = new MyServo(4, 2, 3, 4, 5, 6, 7);
    servos[4] = new MyServo(5, 2, 3, 4, 5, 6, 7);
    servos[5] = new MyServo(6, 2, 3, 4, 5, 6, 7);

    float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
    Kinematic K(geo);

    mock_RobotController R(servos, &K, l2f);
    IOLogic IO;
    AdditionalAxisController A(servos);
    WaitController W;
    MRCPR Mrcpr;

    MRILParser MRILp(R, IO, A, W, Mrcpr);


    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::X, 15));

    // EXPECT_CALL(R, setTargetPose(_,_));


    char instruction[3] = { 'X', '1', '5' };

    MRILp.parse(instruction, 3);
    R.setTargetPose(mock_RobotController::POSITION::X, 15.0);

    releaseArduinoMock();
}
