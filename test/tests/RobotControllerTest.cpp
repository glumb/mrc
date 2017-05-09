#include "gtest/gtest.h"

#include "../fake/util/Logger.h"
#include "Arduino.h"

#include "../fake/Servo.h"
#include "mock_MyServo.h"
#include "MyServo.h"

#include "Kinematic.h"

#include "RobotController.h"


using ::testing::Return;
using ::testing::_;

class RobotControllerTest_MockServo : public ::testing::Test {
protected:

    virtual void SetUp() {
        K = new Kinematic(geo);

        servos[0] = &mock_servos[0];
        servos[1] = &mock_servos[1];
        servos[2] = &mock_servos[2];
        servos[3] = &mock_servos[3];
        servos[4] = &mock_servos[4];
        servos[5] = &mock_servos[5];

        R = new RobotController(servos, *K, logicalAngleLimits, l2p, p2l);
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

    MyServo *servos[6];
    mock_MyServo mock_servos[6];
    Kinematic *K;
    RobotController *R;
};

class RobotControllerTest_MyServo : public ::testing::Test {
protected:

    virtual void SetUp() {
        K = new Kinematic(geo);

        servos[0] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[1] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[2] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[3] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[4] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);
        servos[5] = new MyServo( 1, 100, 500, 1000, -180 * DEG_TO_RAD, 180 * DEG_TO_RAD, 0);

        R = new RobotController(servos, *K, logicalAngleLimits, l2p, p2l);
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

    // MyServo(int          pinNumber,
    //         float        maxAngleVelocity,
    //         unsigned int minFreq,
    //         unsigned int maxFreq,
    //         float        minRadAngle,
    //         float        maxRadAngle,
    //         float        homeRadAngle = 0);
    MyServo *servos[6];

    Kinematic *K;
    RobotController *R;
};

void helper_processServos(MyServo *servos[6]);

void helper_eulerToVector(float euler[3], float vector[3]) {
    float cc = cos(euler[2]);
    float sc = sin(euler[2]);
    float cb = cos(euler[1]);
    float sb = sin(euler[1]);
    float ca = cos(euler[0]);
    float sa = sin(euler[0]);

    vector[0] = sb;
    vector[1] = -sa * cb;
    vector[2] = ca * cb;
}

void helper_moveToTargetPose(RobotController *R, MyServo *servos[6]) {
    while (R->isMoving()) { // move to pose
        R->process();
        helper_processServos(servos);
    }
}

void EULER_EQUALS(float euler1[3], float euler2[3]) {
    float vector1[3];

    helper_eulerToVector(euler1, vector1);
    float vector2[3];
    helper_eulerToVector(euler2, vector2);

    EXPECT_NEAR(vector2[0], vector1[0], 1e-5);
    EXPECT_NEAR(vector2[1], vector1[1], 1e-5);
    EXPECT_NEAR(vector2[2], vector1[2], 1e-5);
}

void helper_processServos(MyServo *servos[6]) {
    for (size_t i = 0; i < 6; i++) {
        servos[i]->process(1000000); // set to target
    }
}

TEST_F(RobotControllerTest_MyServo, test) {
    R->setTargetPose(1, 2, 3, 4, 5, 6);


    // {
    //     InSequence d;
    //   }
    // ArduinoMock *arduinoMock = arduinoMockInstance();
    //
    // EXPECT_CALL(*arduinoMock, digitalWrite(8, HIGH));
    // EXPECT_CALL(*arduinoMock, pinMode(8, OUTPUT));
    //
    // IOLogic iol;
    // iol.setOutput(8, 1);
    // releaseArduinoMock();
}


TEST_F(RobotControllerTest_MockServo, Transaction) {
    R->startTransaction();

    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(45 * DEG_TO_RAD)).Times(0);

    R->setTargetLogicalAngle(0, 45 * DEG_TO_RAD);
    R->process();
    R->process();

    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(45 * DEG_TO_RAD)).Times(1);

    R->endTransaction();
    R->process();
}

TEST_F(RobotControllerTest_MyServo, stop) {
    R->setTargetLogicalAngle(0, 45 * DEG_TO_RAD);
    R->process();

    EXPECT_TRUE(R->isMoving());

    R->stop();

    EXPECT_FALSE(R->isMoving());
}

TEST_F(RobotControllerTest_MyServo, MovementMethod) {
    R->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
    float pose[6];
    R->getCurrentPose(pose);
    pose[0] += 5; // 18.6+5

    EXPECT_FALSE(R->isMoving());

    R->setTargetPose(pose);

    R->process(); // PREPARE_MOVE->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 1.0, 1e-5);

    EXPECT_TRUE(R->isMoving());

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 2.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 3.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 4.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 5.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 6.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 7.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 8.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 9.0, 1e-5);

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 10.0, 1e-5);


    EXPECT_TRUE(R->isMoving());

    R->process(); // MOVING->START_MOVE->MOVING
    helper_processServos(servos);
    R->getCurrentPose(pose);
    EXPECT_NEAR(pose[0], 18.6 + (5.0 / 11.0) * 11.0, 1e-5);

    R->process(); // MOVING->IDLE
    EXPECT_FALSE(R->isMoving());
}

TEST_F(RobotControllerTest_MyServo, getCurrentPose) {
    R->setTargetPose(10, 10, 10, 0, PI, 0);
    R->process();
    helper_processServos(servos);
    float pose[6];
    R->getCurrentPose(pose);

    EXPECT_NEAR(pose[0], 10, 1e-4);
    EXPECT_NEAR(pose[1], 10, 1e-4);
    EXPECT_NEAR(pose[2], 10, 1e-4);
    float euler[3] = { 0, PI, 0 };
    EULER_EQUALS(euler, pose + 3);

    R->setTargetPose(-10, 10, -10, 0, PI, PI);

    // pose does only change after process and updating servos
    EXPECT_NEAR(pose[0], 10, 1e-4);
    EXPECT_NEAR(pose[1], 10, 1e-4);
    EXPECT_NEAR(pose[2], 10, 1e-4);
    EULER_EQUALS(euler, pose + 3);
}
TEST_F(RobotControllerTest_MockServo, getCurrentPose) {
    // pose is fetched from servo
    EXPECT_CALL((mock_servos[0]), getCurrentAngle()).Times(1);
    float pose[6];
    R->getCurrentPose(pose);
}

TEST_F(RobotControllerTest_MyServo, setTargetPose) {}
TEST_F(RobotControllerTest_MyServo, getCurrentLogicalAngles) {}
TEST_F(RobotControllerTest_MyServo, getCurrentLogicalAngle) {}
TEST_F(RobotControllerTest_MyServo, setTargetLogicalAngles) {
    float angles[6] = {
        90 * DEG_TO_RAD,
        110 * DEG_TO_RAD,
        -30 * DEG_TO_RAD,
        -15 * DEG_TO_RAD,
        45 * DEG_TO_RAD,
        130 * DEG_TO_RAD
    };

    R->setTargetLogicalAngles(angles);
    helper_moveToTargetPose(R, servos);
    float currentAngles[6];
    R->getCurrentLogicalAngles(currentAngles);

    EXPECT_NEAR(angles[0], currentAngles[0], 1e-5);
    EXPECT_NEAR(angles[1], currentAngles[1], 1e-5);
    EXPECT_NEAR(angles[2], currentAngles[2], 1e-5);
    EXPECT_NEAR(angles[3], currentAngles[3], 1e-5);
    EXPECT_NEAR(angles[4], currentAngles[4], 1e-5);
    EXPECT_NEAR(angles[5], currentAngles[5], 1e-5);
}
TEST_F(RobotControllerTest_MyServo, setTargetLogicalAnglesMovePastsingularity) {
    float angles[6] = {
        0,
        0,
        0,
        0,
        -90 * DEG_TO_RAD,
        0
    };

    R->setTargetLogicalAngles(angles);
    helper_moveToTargetPose(R, servos);
    float currentAngles[6];
    R->getCurrentLogicalAngles(currentAngles);

    EXPECT_NEAR(angles[0], currentAngles[0], 1e-5);
    EXPECT_NEAR(angles[1], currentAngles[1], 1e-5);
    EXPECT_NEAR(angles[2], currentAngles[2], 1e-5);
    EXPECT_NEAR(angles[3], currentAngles[3], 1e-5);
    EXPECT_NEAR(angles[4], currentAngles[4], 1e-5);
    EXPECT_NEAR(angles[5], currentAngles[5], 1e-5);

    float angles2[6] = {
        0,
        0,
        0,
        0,
        -120 * DEG_TO_RAD,
        0
    };

    R->setTargetLogicalAngles(angles2);
    helper_moveToTargetPose(R, servos);
    float currentAngles2[6];
    R->getCurrentLogicalAngles(currentAngles2);

    EXPECT_NEAR(angles2[0], currentAngles2[0], 1e-5);
    EXPECT_NEAR(angles2[1], currentAngles2[1], 1e-5);
    EXPECT_NEAR(angles2[2], currentAngles2[2], 1e-5);
    EXPECT_NEAR(angles2[3], currentAngles2[3], 1e-5);
    EXPECT_NEAR(angles2[4], currentAngles2[4], 1e-5);
    EXPECT_NEAR(angles2[5], currentAngles2[5], 1e-5);
}
TEST_F(RobotControllerTest_MyServo, setTargetLogicalAngle) {
    R->setTargetLogicalAngle(3, 42 * DEG_TO_RAD);
    helper_moveToTargetPose(R, servos);

    EXPECT_NEAR(R->getCurrentLogicalAngle(3), 42 * DEG_TO_RAD, 1e-7);
}
TEST_F(RobotControllerTest_MyServo, getTargetLogicalAngles) {
    float angles[6] = {
        13,
        99,
        3553,
        1,
        -90 * DEG_TO_RAD,
        292.12
    };

    float currentAngles[6];

    R->setTargetLogicalAngles(angles);
    R->getTargetLogicalAngles(currentAngles);

    EXPECT_NEAR(angles[0], currentAngles[0], 1e-5);
    EXPECT_NEAR(angles[1], currentAngles[1], 1e-5);
    EXPECT_NEAR(angles[2], currentAngles[2], 1e-5);
    EXPECT_NEAR(angles[3], currentAngles[3], 1e-5);
    EXPECT_NEAR(angles[4], currentAngles[4], 1e-5);
    EXPECT_NEAR(angles[5], currentAngles[5], 1e-5);
}
TEST_F(RobotControllerTest_MyServo, getTargetLogicalAngle) {
    R->setTargetLogicalAngle(3, 42 * DEG_TO_RAD);

    EXPECT_NEAR(R->getTargetLogicalAngle(3), 42 * DEG_TO_RAD, 1e-7);
}
TEST_F(RobotControllerTest_MyServo, getCurrentPhysicalAngles) {
    float currentAngles[6];

    R->getCurrentPhysicalAngles(currentAngles);

    EXPECT_NEAR(0, currentAngles[0], 1e-5);
    EXPECT_NEAR(0, currentAngles[1], 1e-5);
    EXPECT_NEAR(0, currentAngles[2], 1e-5);
    EXPECT_NEAR(0, currentAngles[3], 1e-5);
    EXPECT_NEAR(0, currentAngles[4], 1e-5);
    EXPECT_NEAR(0, currentAngles[5], 1e-5);
}
TEST_F(RobotControllerTest_MyServo, getCurrentPhysicalAngle) {
    EXPECT_NEAR(R->getCurrentPhysicalAngle(1), 0 * DEG_TO_RAD, 1e-7);
}

void logical2physical(float angles[6]) {
    angles[2] += angles[1];
}

void physical2logical(float angles[6]) {
    angles[2] -= angles[1];
}

TEST_F(RobotControllerTest_MyServo, LogicalPhysicalAngles) {
    RobotController *Rob;

    Rob = new RobotController(servos, *K, logicalAngleLimits, logical2physical, physical2logical);

    float angles[6] = {
        90 * DEG_TO_RAD,
        110 * DEG_TO_RAD,
        -30 * DEG_TO_RAD,
        -15 * DEG_TO_RAD,
        45 * DEG_TO_RAD,
        130 * DEG_TO_RAD
    };
    Rob->setTargetLogicalAngles(angles);

    helper_moveToTargetPose(Rob, servos);

    float physicalAngles[6];
    float logicalAngles[6];

    Rob->getCurrentLogicalAngles(logicalAngles);
    Rob->getCurrentPhysicalAngles(physicalAngles);

    EXPECT_NEAR(angles[0],                    logicalAngles[0],  1e-5);
    EXPECT_NEAR(angles[1],                    logicalAngles[1],  1e-5);
    EXPECT_NEAR(angles[2],                    logicalAngles[2],  1e-5);
    EXPECT_NEAR(angles[3],                    logicalAngles[3],  1e-5);
    EXPECT_NEAR(angles[4],                    logicalAngles[4],  1e-5);
    EXPECT_NEAR(angles[5],                    logicalAngles[5],  1e-5);

    EXPECT_NEAR(servos[0]->getCurrentAngle(), physicalAngles[0], 1e-5);
    EXPECT_NEAR(servos[1]->getCurrentAngle(), physicalAngles[1], 1e-5);
    EXPECT_NEAR(servos[2]->getCurrentAngle(), physicalAngles[2], 1e-5);
    EXPECT_NEAR(servos[3]->getCurrentAngle(), physicalAngles[3], 1e-5);
    EXPECT_NEAR(servos[4]->getCurrentAngle(), physicalAngles[4], 1e-5);
    EXPECT_NEAR(servos[5]->getCurrentAngle(), physicalAngles[5], 1e-5);
}
TEST_F(RobotControllerTest_MyServo, getTargetPhysicalAngle) {}
TEST_F(RobotControllerTest_MyServo, isMoving) {}
TEST_F(RobotControllerTest_MyServo, process) {}


float length3(float vector[3]) {
    return sqrt(pow(vector[0], 2) +  pow(vector[1], 2) +  pow(vector[2], 2));
}

void cross(float vectorA[3], float  vectorB[3], float result[3]) {
    result[0] = vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1];
    result[1] = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2];
    result[2] = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0];
}

void normalize(float vec[3]) {
    float length = length3(vec);

    vec[0] = vec[0] / length;
    vec[1] = vec[1] / length;
    vec[2] = vec[2] / length;
}

void vecAbs(float vec[3]) {
    vec[0] = fabs(vec[0]);
    vec[1] = fabs(vec[1]);
    vec[2] = fabs(vec[2]);
}

float dot(float vectorA[3], float vectorB[3]) {
    vecAbs(vectorA);
    vecAbs(vectorB);
    normalize(vectorA);
    normalize(vectorB);
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2];
}

void EXPECT_IN_PLANE(float vectors[][3], int length) {
    float normalVectors[length - 1][3];

    for (size_t i = 0; i < length - 1; i++) {
        cross(vectors[i], vectors[i + 1], normalVectors[i]);
    }

    // number of executions int(0.5 * (length - 2) * (length-1)) //little gauss (length - 1) -1

    for (size_t i = 0; i < length - 1; i++) {
        for (size_t j = i + 1; j < length - 1; j++) {
            // expect the |normal vectors| to have almost no angle between them
            EXPECT_NEAR(acos(dot(normalVectors[i], normalVectors[j])), 0, 1e-6);
        }
    }
}

TEST_F(RobotControllerTest_MyServo, linearTransitionBetweenPoses) {
    float vectors[][3] = {
        {  1,      0, 0 },
        {  1,   -7.4, 0 },

        {  0,      1, 0 },
        { 16,      3, 0 },
        {  1,    -20, 0 },
        { 90,     40, 1 },
    };

    float initialPose[6] = { 10, 0, 10, 0, PI, 0 };
    float targetPose[6]  = { 10, -5, 11, 0, PI / 3, 0 };

    R->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);

    R->setTargetPose(initialPose);

    while (R->isMoving()) { // move to pose
        R->process();
        helper_processServos(servos);
    }

    R->setTargetPose(targetPose);

    const int maxSamples = 50; // based on interpolation inerval
    float     interimPoses[maxSamples][3];
    int       i = 0;

    // capture some steps pose to compare the TCP direction
    while (R->isMoving() && maxSamples > i) {
        R->process();
        helper_processServos(servos);
        float pose[6];
        R->getCurrentPose(pose);

        helper_eulerToVector(pose + 3, interimPoses[i]);

        // std::cout << "---------------------------" << '\n';
        // std::cout << interimPoses[i][0] << " " << interimPoses[i][1] << " " << interimPoses[i][2] << '\n';

        i++;
    }

    // EXPECT_IN_PLANE(interimPoses, i - 2);
}


TEST_F(RobotControllerTest_MockServo, dontMoveWhenTargetIsOutOfPhysicalLimits) {}
TEST_F(RobotControllerTest_MockServo, dontMoveWhenTargetIsOutOfLogicalLimits) {
    float lal[6][2] = {
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD },
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD },
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD },
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD },
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD },
        {  -90 * DEG_TO_RAD,  90 * DEG_TO_RAD }
    };
    RobotController *Rob;

    Rob = new RobotController(servos, *K, lal, l2p, p2l);

    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(_)).Times(0);
    Rob->setTargetPose(-10, -10, -10, 0, PI, 0);
    Rob->process();

    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(_)).Times(1);
    Rob->setTargetPose(10, 10, 10, 0, PI, 0);
    Rob->process();


    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(_)).Times(1);
    Rob->setTargetLogicalAngle(0, 80*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(1, 80*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(2, 80*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(3, 80*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(4, 80*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(5, 80*DEG_TO_RAD);
    Rob->process();
    helper_moveToTargetPose(Rob,servos);

    EXPECT_CALL((mock_servos[0]), setTargetRadAngle(_)).Times(0);
    EXPECT_CALL((mock_servos[1]), setTargetRadAngle(_)).Times(0);
    EXPECT_CALL((mock_servos[2]), setTargetRadAngle(_)).Times(0);
    EXPECT_CALL((mock_servos[3]), setTargetRadAngle(_)).Times(0);
    Rob->setTargetLogicalAngle(0, 91*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(1, 91*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(2, 91*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(3, 91*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(4, 91*DEG_TO_RAD);
    Rob->setTargetLogicalAngle(5, 91*DEG_TO_RAD);
    Rob->process();
    EXPECT_FALSE(Rob->isMoving());
    helper_moveToTargetPose(Rob,servos);

    float targetAngles[6];
    Rob->getTargetLogicalAngles(targetAngles);
    EXPECT_NEAR(targetAngles[0], 91*DEG_TO_RAD, 1e-5);
    EXPECT_NEAR(targetAngles[1], 91*DEG_TO_RAD, 1e-5);
    EXPECT_NEAR(targetAngles[2], 91*DEG_TO_RAD, 1e-5);
    EXPECT_NEAR(targetAngles[3], 91*DEG_TO_RAD, 1e-5);
    EXPECT_NEAR(targetAngles[4], 91*DEG_TO_RAD, 1e-5);
    EXPECT_NEAR(targetAngles[5], 91*DEG_TO_RAD, 1e-5);
}
