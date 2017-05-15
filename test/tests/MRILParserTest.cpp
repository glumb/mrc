#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Arduino.h"

static SerialMock *serialMock = serialMockInstance();

#include "mock_RobotController.h"
#include "mock_MRCPR.h"

#include "../fake/fastRW.h"
#include "../fake/Communication.h"

#include "MRCPR.h"
#include "MyServo.h"

#include "Kinematic.h"

#include "MRILParser.h"

#include "IOLogic.h"

#include "AdditionalAxisController.h"

using ::testing::InSequence;
using ::testing::_;

void l2p(float f[]) {}
void p2l(float f[]) {}

MyServo *servos[6] = {
    new MyServo(1, 2, 3, 4, 5, 6, 7),
    new MyServo(2, 2, 3, 4, 5, 6, 7),
    new MyServo(3, 2, 3, 4, 5, 6, 7),
    new MyServo(4, 2, 3, 4, 5, 6, 7),
    new MyServo(5, 2, 3, 4, 5, 6, 7),
    new MyServo(6, 2, 3, 4, 5, 6, 7)
};

float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };
Kinematic K(geo);

float logicalAngleLimits[6][2] = {
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
    { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD }
};

static unsigned int pinMap[10] = { 8, 11, 12, 7, 0, 1, 0, 0, 0, 0 };

CommunicationInterface Com;
IOLogic IO(pinMap);
AdditionalAxisController A(servos);
WaitController W;
MRCPR Mrcpr(Com);

using ::testing::_;


TEST(MRILParserTest, setOutputToHigh)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser   MRILp(R, IO, A, W, Mrcpr);
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalWrite(pinMap[3], HIGH));
    EXPECT_CALL(*arduinoMock, pinMode(pinMap[3], OUTPUT));

    char instruction[3] = { 'O', '3', '1' };

    MRILp.parse(instruction, 3);

    releaseArduinoMock();
}

TEST(MRILParserTest, waitForInput)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser   MRILp(R, IO, A, W, Mrcpr);
    ArduinoMock *arduinoMock = arduinoMockInstance();

    EXPECT_CALL(*arduinoMock, digitalRead(pinMap[4]));
    EXPECT_CALL(*arduinoMock, pinMode(pinMap[4], INPUT));

    char instruction[3] = { 'I', '4', '1' };

    MRILp.parse(instruction, 3);
    MRILp.process();

    releaseArduinoMock();
}


TEST(MRILParserTest, setX)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::X, 15));

    char instruction[5] = { 'X', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, setY)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::Y, 15));

    char instruction[5] = { 'Y', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, setZ)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::Z, 15));

    char instruction[5] = { 'Z', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, setA)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::A, 15*DEG_TO_RAD));

    char instruction[5] = { 'A', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, setB)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::B, 15*DEG_TO_RAD));


    char instruction[5] = { 'B', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, setC)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::C, 15*DEG_TO_RAD));


    char instruction[5] = { 'C', '1', '5', '.', '0' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, getXYZABC)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    {
        InSequence d;
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::X));
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::Y));
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::Z));
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::A));
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::B));
        EXPECT_CALL(R, getCurrentPose(mock_RobotController::POSITION::C));
    }

    char instruction[6] = { 'X', 'Y', 'Z', 'A', 'B', 'C' };

    MRILp.parse(instruction, 6);
}

TEST(MRILParserTest, setMultipleCoordinates)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);
    {
        InSequence d;
        EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::Z, 3)).Times(1).RetiresOnSaturation();
        EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::A, 4*DEG_TO_RAD)).Times(1).RetiresOnSaturation();
        EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::B, 5*DEG_TO_RAD)).Times(1).RetiresOnSaturation();
        EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::C, 6*DEG_TO_RAD)).Times(1).RetiresOnSaturation();
    }

    char instruction[8] = { 'Z', '3', 'A', '4', 'B', '5', 'C', '6' };

    MRILp.parse(instruction, 8);
}

TEST(MRILParserTest, setNegativeNumbers)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::Y, -2));


    char instruction[5] = { 'Y', '-', '0', '0', '2' };
    MRILp.parse(instruction, 5);
}

TEST(MRILParserTest, dontParseComments)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetPose(mock_RobotController::POSITION::X, 2)).Times(0);

    char instruction[3] = { '#', 'X', '2' };
    MRILp.parse(instruction,  3);

    char instruction2[3] = { '(', 'X', '2' };
    MRILp.parse(instruction2, 3);
}

TEST(MRILParserTest, setVelocity)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setMaxVelocity(12)).Times(1);

    char instruction[3] = { 'V', '1', '2' };
    MRILp.parse(instruction, 3);
}

TEST(MRILParserTest, getVelocity)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, getMaxVelocity()).Times(2);

    char instruction[2] = { 'V', 'V' };
    MRILp.parse(instruction, 2);
}

TEST(MRILParserTest, setMovementMethod)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR)).Times(1);
    EXPECT_CALL(R, setMovementMethod(RobotController::MOVEMENT_METHODS::P2P)).Times(1);

    char instruction[3] = { 'M', '0', '1' };
    MRILp.parse(instruction,  3);
    char instruction2[3] = { 'M', '0', '0' };
    MRILp.parse(instruction2, 3);
}

TEST(MRILParserTest, getMovementMethod)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, getMovementMethod()).Times(1);

    char instruction[1] = { 'M' };
    MRILp.parse(instruction, 1);
}

TEST(MRILParserTest, setAngle)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, setTargetLogicalAngle(2, 90 * DEG_TO_RAD)).Times(1);
    EXPECT_CALL(R, setTargetLogicalAngle(4, -10 * DEG_TO_RAD)).Times(1);

    char instruction[] = { 'R', '2', '9', '0' };
    MRILp.parse(instruction,  4);
    char instruction2[] = { 'R', '4', '-', '1', '0' };
    MRILp.parse(instruction2, 5);
}

TEST(MRILParserTest, getAngle)
{
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(R, getCurrentLogicalAngle(3)).Times(1);

    char instruction[2] = { 'R', '3' };
    MRILp.parse(instruction, 2);
}

TEST(MRILParserTest, setWait)
{
    ArduinoMock *arduinoMock = arduinoMockInstance();

    arduinoMock->setMillisRaw(1200);

    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, Mrcpr);

    EXPECT_CALL(*arduinoMock, millis()).Times(3); // 1 per 'isDone' and 1 per setWaitMs

    EXPECT_TRUE(W.isDone());

    char instruction[3] = { 'D', '4', '2' };
    MRILp.parse(instruction, 3);

    EXPECT_FALSE(W.isDone());

    releaseArduinoMock();
}

TEST(MRILParserTest, sendExecutingCommandNumber)
{
    ArduinoMock *arduinoMock = arduinoMockInstance();

    mock_MRCPR MrcprM(Com);
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, MrcprM);

    EXPECT_CALL(MrcprM, sendMessage(String("N042")));
    EXPECT_CALL(MrcprM, sendMessage(String("N142"))).Times(0);

    char instruction[] = { 'N', '4', '2', 'X', '5' };
    MRILp.parse(instruction, 5);

    releaseArduinoMock();
}

TEST(MRILParserTest, sendExecutedCommandNumber)
{
    ArduinoMock *arduinoMock = arduinoMockInstance();

    mock_MRCPR MrcprM(Com);
    mock_RobotController R(servos, K,logicalAngleLimits, l2p, p2l);
    MRILParser MRILp(R, IO, A, W, MrcprM);

    {
        InSequence d;
        EXPECT_CALL(MrcprM, sendMessage(String("N042")));
        EXPECT_CALL(MrcprM, sendMessage(_)); // return X value
        EXPECT_CALL(MrcprM, sendMessage(String("N142")));
    }

    char instruction[] = { 'N', '4', '2', 'X' };

    MRILp.parse(instruction, 4);

    releaseArduinoMock();
}
