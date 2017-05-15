#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Arduino.h"

static SerialMock *serialMock = serialMockInstance();
#include "EEPROM.h"



#include "mock_RobotController.h"
#include "mock_MRCPR.h"
#include "mock_MRILParser.h"

#include "../fake/fastRW.h"
#include "../fake/Communication.h"

#include "EEPromStorage.h"
#include "RingBuffer.h"
#include "MRCPR.h"
#include "VarSpeedServo.h"

#include "Kinematic.h"

#include "MRCPParser.h"

#include "IOLogic.h"

#include "AdditionalAxisController.h"


using ::testing::InSequence;
using ::testing::_;

static CommunicationInterface Com;
static MRCPR Mrcpr(Com);
static     WaitController W;

static unsigned int pinMap[10] = { 8, 11, 12, 7, 0, 1, 0, 0, 0, 0 };

class MRCPParserTest : public ::testing::Test {
protected:

    virtual void SetUp() {
        K = new Kinematic(geo);

        servos[0] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[1] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[2] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[3] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[4] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[5] = new VarSpeedServo(1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);


        R = new mock_RobotController(servos, *K, logicalAngleLimits, l2p, p2l);

        A = new AdditionalAxisController(servos + 6);
    }

    virtual void TearDown()     {}

    static void  l2p(float f[]) {}

    static void  p2l(float f[]) {}

    float geo[5][3] = {
        {    5, 0,  7.3 },
        {    0, 0, 13.0 },
        {    1, 0,    2 },
        { 12.6, 0,    0 },
        {    0, 0, -3.6 }
    };

    float logicalAngleLimits[6][2] = {
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD },
        { -180 * DEG_TO_RAD, 180 * DEG_TO_RAD }
    };

    // VarSpeedServo(int          pinNumber,
    //         float        maxAngleVelocity,
    //         unsigned int minFreq,
    //         unsigned int maxFreq,
    //         float        minRadAngle,
    //         float        maxRadAngle,
    //         float        homeRadAngle = 0);
    VarSpeedServo *servos[6];

    Kinematic *K;
    mock_RobotController *R;


    IOLogic IO {pinMap};
    AdditionalAxisController *A;


    EEPromStorage Eepromstorage;
    RingBuffer Ringbuffer { 200 };
};

static void helper_processServos(VarSpeedServo *servos[6]) {
  for (size_t i = 0; i < 6; i++) {
    servos[i]->process(1000000); // set to target
  }
}
static void helper_moveToTargetPose(RobotController *R, VarSpeedServo *servos[6]) {
    while (R->isMoving()) { // move to pose
        R->process();
        helper_processServos(servos);
    }
}




TEST_F(MRCPParserTest, queueIn)
{
  EEPROMMock *mock     = EEPROMMockInstance(); // mrcp looks for data in eeprom on init
    ArduinoMock *arduinoMock = arduinoMockInstance(); // need the mock,since millis is calles
    arduinoMock->setMillisRaw(2000);
    mock_MRILParser mrilparser(*R,
                               IO,
                               *A,
                               W,
                               Mrcpr);

    MRCPParser mrcpparser(Eepromstorage,
                          Ringbuffer,
                          mrilparser,
                          Mrcpr);

    char message[] = { MRCP_START_FRAME, MRCP_COMMAND_QUEUE_IN, 'X', '1', '5', MRCP_END_FRAME, MRCP_START_FRAME, MRCP_COMMAND_QUEUE_IN, 'Y', '1', '5', MRCP_END_FRAME };

    EXPECT_CALL(mrilparser, parse(_, _)).Times(1);

    for (size_t i = 0; i < 12; i++) {
        mrcpparser.parseChar(message[i]);
    }
    helper_moveToTargetPose(R,servos);
    mrilparser.process();
    mrcpparser.process();

}

TEST_F(MRCPParserTest, execute)
{
    mock_MRILParser mrilparser(*R,
                               IO,
                               *A,
                               W,
                               Mrcpr);

    MRCPParser mrcpparser(Eepromstorage,
                          Ringbuffer,
                          mrilparser,
                          Mrcpr);

    char message[] = { MRCP_START_FRAME, MRCP_COMMAND_EXECUTE, 'X', '1', '5', MRCP_END_FRAME, MRCP_START_FRAME, MRCP_COMMAND_EXECUTE, 'Y', '1', '5', MRCP_END_FRAME };

    EXPECT_CALL(mrilparser, parse(_, _)).Times(2);

    for (size_t i = 0; i < 12; i++) {
        mrcpparser.parseChar(message[i]);
    }
}
