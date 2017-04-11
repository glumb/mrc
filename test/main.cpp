#include "gtest/gtest.h"
#include "KinematicTest.cpp"

#ifndef TEST
#define TEST 1

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
