#include "servodata.h"

// geometry
float geometry[5][3] = {
    {  4.2, 0,   8.5 },
    {    0, 0,   9.5 },
    {    1, 0,  1.85 },
    { 14.8, 0,     0 },
    {    0, 0,  -2.2 }
};

// E.g. joint 0 cant be < 90° to not crash into itself
float logicAngleLimits[6][2] = {
    { servoConfig[0][4],
      servoConfig[0][5] },
    { servoConfig[1][4],
      servoConfig[1][5] },
    {  -25 * DEG_TO_RAD,
       48 * DEG_TO_RAD  },
    { servoConfig[3][4],
      servoConfig[3][5] },
    { servoConfig[4][4],
      servoConfig[4][5] },
    { servoConfig[5][4],
      servoConfig[5][5] }
};

float v_1 = 9.5;
float h_1 = 2.619;
float h_2 = 10.29;
float h_3 = 3.3;

// relation between physical and logical angles based on robot kinematic coupling.
void logicalToPhysicalAngles(float angles[6]) {
    float alpha  = angles[2];
    float alpha_ = alpha + PI / 2;
    float diag_2 = sqrt(v_1 * v_1 - 2 * v_1 * h_3 * cos(alpha_) + h_3 * h_3);
    float beta_1 = acos((-h_2 * h_2 + h_1 * h_1 + diag_2 * diag_2) / (2 * h_1 * diag_2));
    float beta_2 = acos((-h_3 * h_3 + v_1 * v_1 + diag_2 * diag_2) / (2 * v_1 * diag_2));

    angles[2] = PI / 2 - (beta_1 + beta_2);

    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];

    float beta    = angles[2];
    float beta_   = (PI / 2 - beta);
    float diag_1  = sqrt(v_1 * v_1 - 2 * v_1 * h_1 * cos(beta_) + h_1 * h_1);
    float alpha_1 = acos((-h_2 * h_2 + h_3 * h_3 + diag_1 * diag_1) / (2 * h_3 * diag_1));
    float alpha_2 = acos((-h_1 * h_1 + v_1 * v_1 + diag_1 * diag_1) / (2 * v_1 * diag_1));
    angles[2] = alpha_1 + alpha_2 - PI / 2;
}

// configure additional axis such as grippers and other devices
// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position || angleDegMin/Max can be any value you
// want to map the min and max frequency to.
// It may be usefull to map a gripper to 0-100 based on the percentage of opening.
const float additionalAxisServoConfig[2][7] = {
    { pin_additional_servo_6, 160.00 * DEG_TO_RAD, 1888.00,    1122,    0.00 * DEG_TO_RAD,  1.00 * DEG_TO_RAD, 0 }, // mapped to 0-1, to
                                                                                                                    // open:1, closed:0
    { pin_additional_servo_7, 160.00 * DEG_TO_RAD,    1000, 2000.00,  -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0 }
};

unsigned int pinMap[10] = { 8, 11, 12, 7, 0, 1, 0, 0, 0, 0 }; // todo
