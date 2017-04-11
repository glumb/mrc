#include "gtest/gtest.h"
#include "Kinematic.h"

#ifndef KINEMATIC_TEST_H
# define KINEMATIC_TEST_H

// #include "Kinematic.h" uncomment include_dir
// #include "../src/Kinematic.h"
// #include "gtest/gtest.h"
// #include "gmock/gmock.h"

namespace {
float geo[5][3] = { { 3.5, 8.5, 0 }, { 0, 11.6, 0 }, { 1.4, 1.5, 0 }, { 12, 0, 0 }, { 0, -5, 0 } };

Kinematic Kin(geo);

TEST(KinematicTest, forwardEqualsRevers) {
    float initialAngles[6] = { 0, 0, 0, 0, 0, 0 };

    float pose[6];

    Kin.forward(initialAngles[0], initialAngles[1], initialAngles[2], initialAngles[3], initialAngles[4], initialAngles[5], pose);

    float angles[6];
    Kin.inverse(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], angles);

    for (size_t i = 0; i < 6; i++) {
        EXPECT_NEAR(initialAngles[i], angles[i], 0.000001);
    }
}
}

#endif // ifndef KINEMATIC_TEST_H
