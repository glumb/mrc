#include "gtest/gtest.h"

#include "Arduino.h"

#include "Kinematic.h"


namespace {
float geo[5][3] = {
    {    5, 0,  7.3 },
    {    0, 0, 13.0 },
    {    1, 0,    2 },
    { 12.6, 0,    0 },
    {    0, 0, -3.6 }
};

Kinematic Kin(geo);



void eulerToVectorHelper(float euler[3], float vector[3]) {
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

void EULER_EQUALS(float euler1[3],float euler2[3]) {
  float vector1[3];
  eulerToVectorHelper(euler1,vector1);
  float vector2[3];
  eulerToVectorHelper(euler2,vector2);

  EXPECT_NEAR(vector2[0], vector1[0], 1e-5);
  EXPECT_NEAR(vector2[1], vector1[1], 1e-5);
  EXPECT_NEAR(vector2[2], vector1[2], 1e-5);
}

TEST(KinematicTest, forwardHome) {
    float initialAngles[6] = { 0, 0, 0, 0, 0, 0 };

    float pose[6];

    Kin.forward(initialAngles[0], initialAngles[1], initialAngles[2], initialAngles[3], initialAngles[4], initialAngles[5], pose);

    for (size_t i = 0; i < 3; i++) {
        EXPECT_NEAR(geo[0][i] + geo[1][i] + geo[2][i] + geo[3][i] + geo[4][i], pose[i], 0.000001);
    }
}

TEST(KinematicTest, forward) {
    float initialAngles[6] = { 0, 0, -PI / 2.0, 0, 0, 0 };

    float pose[6];

    Kin.forward(initialAngles[0], initialAngles[1], initialAngles[2], initialAngles[3], initialAngles[4], initialAngles[5], pose);
    float x = geo[0][0] + geo[1][0] - geo[2][2] - geo[3][2] - geo[4][2];
    float y = geo[0][1] + geo[1][1] + geo[2][1] + geo[3][1] + geo[4][1];
    float z = geo[0][2] + geo[1][2] + geo[2][0] + geo[3][0] + geo[4][0];


    EXPECT_NEAR(x, pose[0], 0.000001);
    EXPECT_NEAR(y, pose[1], 0.000001);
    EXPECT_NEAR(z, pose[2], 0.000001);
}

TEST(KinematicTest, inverseHome) {
    float pose[3];

    for (size_t i = 0; i < 3; i++) {
        pose[i] = geo[0][i] + geo[1][i] + geo[2][i] + geo[3][i] + geo[4][i];
    }


    float angles[6];
    Kin.inverse(pose[0], pose[1], pose[2], 0, PI, 0, angles);

    for (size_t i = 0; i < 6; i++) {
        EXPECT_NEAR(0, angles[i], 0.000001);
    }
}

TEST(KinematicTest, forwardEqualsInverseHome) {
    float initialAngles[6] = { 0, 0, 0, 0, 0, 0 };

    float pose[6];

    Kin.forward(initialAngles[0], initialAngles[1], initialAngles[2], initialAngles[3], initialAngles[4], initialAngles[5], pose);

    float angles[6];
    Kin.inverse(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], angles);

    for (size_t i = 0; i < 6; i++) {
        EXPECT_NEAR(initialAngles[i], angles[i], 0.000001);
    }
}

TEST(KinematicTest, forwardEqualsInverse) {
    float testPoses[4][6] = {
        { 10, 10, 10,       0, PI,                  0 },
        { 10, 10, 10,       0, PI,     PI             },
        { 10, 10, 10, -PI / 2, PI,     PI / 3         },
        { 10, 10, 10,       0, PI / 2,              0 }
    };

    for (size_t i = 0; i < 4; i++) {
        float angles[6];
        Kin.inverse(testPoses[i][0], testPoses[i][1], testPoses[i][2], testPoses[i][3], testPoses[i][4], testPoses[i][5], angles);

        float pose[6];
        Kin.forward(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], pose);

        for (size_t j = 0; j < 3; j++) {
            EXPECT_NEAR(testPoses[i][j], pose[j], 1e-5);
        }

        float testPose[3] = { testPoses[i][3], testPoses[i][4], testPoses[i][5] };

       EULER_EQUALS(pose + 3, testPose);
    }
}
}
