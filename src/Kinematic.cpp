#include "Kinematic.h"
#include <Arduino.h>
#include <math.h>
#include "Logger.h"

const unsigned int Kinematic::OK           = 0;
const unsigned int Kinematic::OUT_OF_RANGE = 1;

namespace {
Logger logger = Logger("Kinematic");
}

void log(String text) {
    logger.info(text);
}

Kinematic::Kinematic(float geometry[5][3], ROBOT_TYPE robotType) {
    this->robotType = robotType;

    this->V1_length_x_y   =  sqrt(pow(geometry[1][0], 2) +  pow(geometry[1][1], 2));
    this->V4_length_x_y_z =  sqrt(pow(geometry[4][0], 2) +  pow(geometry[4][1], 2) +  pow(geometry[4][2], 2));

    for (int i = 0; i < 5; i++) {
        this->geometry[i][0] = geometry[i][0];
        this->geometry[i][1] = geometry[i][1];
        this->geometry[i][2] = geometry[i][2];
    }

    float tmpPos[3] = { 0, 0, 0 };

    for (int i = 0; i < 5; i++) {
        this->J_initial_absolute[i][0] = tmpPos[0];
        this->J_initial_absolute[i][1] = tmpPos[1];
        this->J_initial_absolute[i][2] = tmpPos[2];
        tmpPos[0]                     += geometry[i][0];
        tmpPos[1]                     += geometry[i][1];
        tmpPos[2]                     += geometry[i][2];
    }

    for (int i = 0; i < 6; i++) {
        this->A_corrected[i] = 0;
    }

    this->A_corrected[1] -=  PI / 2.0;
    this->A_corrected[1] +=  atan2(geometry[1][0], geometry[1][1]);                                       // correct offset bone

    this->A_corrected[2] -=  PI / 2.0;
    this->A_corrected[2] -=  atan2((geometry[2][1] + geometry[3][1]), (geometry[2][0] + geometry[3][0])); // correct offset bone V2,V3
    this->A_corrected[2] -=  atan2(geometry[1][0], geometry[1][1]);                                       // correct bone offset of V1

    // this->A_corrected[4] +=  PI / 2;
    this->A_corrected[4] +=  atan2(geometry[4][1], geometry[4][0]);

    // console.log("---------------------------------" +  atan2(geometry[4][1], geometry[4][0]))
}

// takes ~11 ms on Atmega328p nano ~6ms without println
int Kinematic::inverse(float x, float y, float z, float a, float b, float c, float angles[6]) {
    // console.log(x, y, z, a, b, c)


    float ca = cos(a);
    float sa = sin(a);
    float cb = cos(b);
    float sb = sin(b);
    float cc = cos(c);
    float sc = sin(c);

    float targetVectorX[3] = {
        cb *cc,
        cb * sc,
        -sb,
    };

    if (this->robotType == ROBOT_TYPE::AXIS4) {
        if ((abs(targetVectorX[0]) > 0.0001) || (abs(targetVectorX[1] + 1) > 0.0001) ||
            (abs(targetVectorX[2]) > 0.0001)) {
            logger.warning("cant reach pose, TCP axis must be vertical");

            // return Kinematic::OUT_OF_RANGE; todo or use this
        }
        targetVectorX[0] = 0;
        targetVectorX[0] = -1;
        targetVectorX[0] = 0;
    }

    float R[6] = {
        this->A_corrected[0],
        this->A_corrected[1],
        this->A_corrected[2],
        this->A_corrected[3],
        this->A_corrected[4],
        this->A_corrected[5]
    };


    log("X " + String(x));
    log("Y " + String(y));
    log("Z " + String(z));

    log("A " + String(a));
    log("B " + String(b));
    log("C " + String(c));

    log("A0 corrected " + String(R[0]));
    log("A1 corrected " + String(R[1]));
    log("A2 corrected " + String(R[2]));
    log("A3 corrected " + String(R[3]));
    log("A4 corrected " + String(R[4]));
    log("A5 corrected " + String(R[5]));


    float J[6][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };


    // ---- J5 ----

    J[5][0] = x;
    J[5][1] = y;
    J[5][2] = z;

    log("J5 " + String(J[5][0]) + ", " + String(J[5][1]) + ", " + String(J[5][2]));

    // ---- J4 ----
    // targetVectorX

    J[4][0] = x - this->V4_length_x_y_z * targetVectorX[0];
    J[4][1] = y - this->V4_length_x_y_z * targetVectorX[1];
    J[4][2] = z - this->V4_length_x_y_z * targetVectorX[2];

    log("J4 " + String(J[4][0]) + ", " + String(J[4][1]) + ", " + String(J[4][2]));

    // todo backwards rotation


    // ---- A0 ----
    // # J4

    R[0] +=  PI / 2 -  acos(this->J_initial_absolute[4][2] / this->length2(J[4][2], J[4][0]));
    R[0] +=  atan2(-J[4][2], J[4][0]);

    if (this->J_initial_absolute[4][2] > this->length2(J[4][2], J[4][0])) {
        log("out of reach");
    }

    //                plane.rotation.y = R[0]


    // ---- J1 ----
    // # A0

    J[1][0] =  cos(R[0]) * this->geometry[0][0] +  sin(R[0]) * this->geometry[0][2];
    J[1][1] = this->geometry[0][1];
    J[1][2] = -sin(R[0]) * this->geometry[0][0] +  cos(R[0]) * this->geometry[0][2];

    log("J1 " + String(J[1][0]) + ", " + String(J[1][1]) + ", " + String(J[1][2]));

    // ---- rotate J4 into x,y plane ----
    // # J4 A0

    float J4_x_y[3];

    J4_x_y[0] =  cos(R[0]) * J[4][0] + -sin(R[0]) * J[4][2];
    J4_x_y[1] = J[4][1];
    J4_x_y[2] =  sin(R[0]) * J[4][0] +  cos(R[0]) * J[4][2];


    // ---- J1J4_projected_length_square ----
    // # J4 A0

    float J1J4_projected_length_square =  pow(J4_x_y[0] - this->J_initial_absolute[1][0], 2) +  pow(
        J4_x_y[1] - this->J_initial_absolute[1][1],
        2); // not using  sqrt
    log("J4_x_y[0]" + String(J4_x_y[0]));
    log("this->J_initial_absolute[1][0]" + String(this->J_initial_absolute[1][0]));
    log("J4_x_y[1]" + String(J4_x_y[1]));
    log("this->J_initial_absolute[1][1]" + String(this->J_initial_absolute[1][1]));

    // ---- A2 ----
    // # J4 A0

    float J2J4_length_x_y = this->length2(this->geometry[2][0] + this->geometry[3][0], this->geometry[2][1] + this->geometry[3][1]);
    log("J2J4_length_x_y" + String(J2J4_length_x_y));
    log("J1J4_projected_length_square" + String(J1J4_projected_length_square));
    R[2] +=
        acos((-J1J4_projected_length_square +
              pow(J2J4_length_x_y, 2) +  pow(this->V1_length_x_y, 2)) / (2.0 * (J2J4_length_x_y) * this->V1_length_x_y));


    // ---- A1 ----
    // # J4 A0

    float J1J4_projected_length =  sqrt(J1J4_projected_length_square);
    R[1] +=  atan2((J4_x_y[1] - this->J_initial_absolute[1][1]), (J4_x_y[0] - this->J_initial_absolute[1][0]));
    R[1] +=
        acos((+J1J4_projected_length_square -
              pow(J2J4_length_x_y, 2) +  pow(this->V1_length_x_y, 2)) / (2.0 * J1J4_projected_length * this->V1_length_x_y));


    // ---- J2 ----
    // # A1 A0

    float ta =  cos(R[0]);
    float tb =  sin(R[0]);
    float tc = this->geometry[0][0];
    float d  = this->geometry[0][1];
    float e  = this->geometry[0][2];
    float f  =  cos(R[1]);
    float g  =  sin(R[1]);
    float h  = this->geometry[1][0];
    float i  = this->geometry[1][1];
    float j  = this->geometry[1][2];
    float k  =  cos(R[2]);
    float l  =  sin(R[2]);
    float m  = this->geometry[2][0];
    float n  = this->geometry[2][1];
    float o  = this->geometry[2][2];


    J[2][0] = ta * tc + tb * e + ta * f * h - ta * g * i + tb * j;
    J[2][1] = d + g * h + f * i;
    J[2][2] = -tb * tc + ta * e - tb * f * h + tb * g * i + ta * j;


    // ---- J3 ----
    // # A0 A1 A2


    J[3][0] = ta * tc + tb * e + ta * f * h - ta * g * i + tb * j + ta * f * k * m - ta * g * l * m - ta * g * k * n - ta * f * l * n + tb *
              o;
    J[3][1] = d + g * h + f * i + g * k * m + f * l * m + f * k * n - g * l * n;
    J[3][2] = -tb * tc + ta * e - tb * f * h + tb * g * i + ta * j - tb * f * k * m + tb * g * l * m + tb * g * k * n + tb * f * l * n +
              ta *
              o;


    // ---- J4J3 J4J5 ----
    // # J3 J4 J5

    float J4J5_vector[3] = { J[5][0] - J[4][0], J[5][1] - J[4][1], J[5][2] - J[4][2] };
    float J4J3_vector[3] = { J[3][0] - J[4][0], J[3][1] - J[4][1], J[3][2] - J[4][2] };


    // ---- A3 ----
    // J3 J4 J5

    float J4J5_J4J3_normal_vector[3];

    this->cross(J4J5_vector, J4J3_vector, J4J5_J4J3_normal_vector);


    float XZ_parallel_aligned_vector[3] = { 10 *  cos(R[0] +  PI / 2),
                                            0,
                                            -10 *  sin(R[0] +  PI / 2) };

    float reference[3];
    this->cross(XZ_parallel_aligned_vector, J4J3_vector, reference);

    // todo dont use coordinate aligned angle - may result in turns when y < 0 -> axis 4 goes outwards
    R[3] = this->angleBetween(J4J5_J4J3_normal_vector, XZ_parallel_aligned_vector, reference);


    // ---- A4 ----
    // #J4 J3 J5

    float reference_vector[3];
    this->cross(J4J3_vector, J4J5_J4J3_normal_vector, reference_vector);

    R[4] += this->angleBetween(J4J5_vector, J4J3_vector, reference_vector);

    // static configuration
    // if (R[3] > PI / 2) {
    //     R[3] -= PI;
    //
    //     if (R[4] > PI / 2) {
    //         // R[4] = R[4] + PI
    //         R[4] = PI / 2 - (R[4] - PI / 2);
    //     } else {
    //         // R[4] -R[4] + PI
    //         R[4] = PI / 2 + (PI / 2 - R[4]);
    //     }
    // } else if (R[3] < -PI / 2) {
    //     R[3] += PI;
    //
    //     if (R[4] > PI / 2) {
    //         R[4] = PI / 2 - (R[4] - PI / 2);
    //     } else {
    //         R[4] = PI / 2 + (PI / 2 - R[4]);
    //     }
    // }
    //
    if (R[3] > PI / 2.0) {
        R[3] -= PI;
        R[5] -= PI;

        if (R[4] > PI / 2.0) {
            // R[4] = R[4] + PI
            R[4] = PI / 2.0 - (R[4] - PI / 2.0);
        } else {
            // R[4] -R[4] + PI
            R[4] = PI / 2.0 + (PI / 2.0 - R[4]);
        }
    } else if (R[3] < -PI / 2.0) {
        R[3] += PI;
        R[5] += PI;

        if (R[4] > PI / 2.0) {
            R[4] = PI / 2.0 - (R[4] - PI / 2.0);
        } else {
            R[4] = PI / 2.0 + (PI / 2.0 - R[4]);
        }
    }

    R[5] = fmod((-PI + (fmod((R[5] + PI), (2 * PI)))), (2 * PI));

    // ---- A5 ----

    float targetVectorY[3] = {
        sa *sb *cc - sc * ca,
        sa * sb * sc + cc * ca,
        sa * cb,
    };

    float cross_targetVectorX_targetVectorY[3];
    this->cross(targetVectorX, targetVectorY, cross_targetVectorX_targetVectorY);

    // float cross_targetVectorY_targetVectorX[3];
    // this->cross(targetVectorY, targetVectorX, cross_targetVectorY_targetVectorX);

    // R[5] += PI / 2;
    // R[5] -= this->angleBetween(J4J5_J4J3_normal_vector, targetVectorY, cross_targetVectorY_targetVectorX);

    R[5] += PI / 2.0;
    R[5] -= this->angleBetween(J4J5_J4J3_normal_vector, targetVectorY, cross_targetVectorX_targetVectorY);

    if (R[5] > PI) {
        R[5] = -PI + (fmod(R[5], PI));
    }

    bool error = false;

    for (int ij = 0; ij < 6; ij++) {
        if (isnan(R[ij])) {
            log("E A_" + String(ij) + " out of reach ");
            error = true;
        }
    }

    if (!error) {
        log("A0 " + String(R[0]));
        log("A1 " + String(R[1]));
        log("A2 " + String(R[2]));
        log("A3 " + String(R[3]));
        log("A4 " + String(R[4]));
        log("A5 " + String(R[5]));


        log("J0 X" + String(J[0][0]) + " Y" + String(J[0][1]) + " Z" + String(J[0][2]));
        log("J1 X" + String(J[1][0]) + " Y" + String(J[1][1]) + " Z" + String(J[1][2]));
        log("J2 X" + String(J[2][0]) + " Y" + String(J[2][1]) + " Z" + String(J[2][2]));
        log("J4 X" + String(J[4][0]) + " Y" + String(J[4][1]) + " Z" + String(J[4][2]));
        log("J5 X" + String(J[5][0]) + " Y" + String(J[5][1]) + " Z" + String(J[5][2]));

        log("J5 A " + String(a) + " B " + String(b) + " C " + String(c));

        angles[0] = R[0];
        angles[1] = R[1];
        angles[2] = R[2];
        angles[3] = R[3];
        angles[4] = R[4];
        angles[5] = R[5];

        return Kinematic::OK;
    } else {
        return Kinematic::OUT_OF_RANGE;
    }
}

void Kinematic::forward(float A0, float A1, float A2, float A3, float A4, float A5, float jointsResult[6]) {
    float TCP[7][3];

    logger.info("--forward--");
    logger.info("A0" + String(A0));
    logger.info("A1" + String(A1));
    logger.info("A2" + String(A2));
    logger.info("A3" + String(A3));
    logger.info("A4" + String(A4));
    logger.info("A5" + String(A5));

    this->calculateCoordinates(A0, A1, A2, A3, A4, A5, TCP);

    logger.info("--forward-TCP--");
    logger.info("x" + String(TCP[5][0]));
    logger.info("y" + String(TCP[5][1]));
    logger.info("z" + String(TCP[5][2]));
    logger.info("a" + String(TCP[6][0]));
    logger.info("b" + String(TCP[6][1]));
    logger.info("c" + String(TCP[6][2]));

    jointsResult[0] = TCP[5][0];
    jointsResult[1] = TCP[5][1];
    jointsResult[2] = TCP[5][2];
    jointsResult[3] = TCP[6][0];
    jointsResult[4] = TCP[6][1];
    jointsResult[5] = TCP[6][2];
}

void Kinematic::calculateCoordinates(float R0, float R1, float R2, float R3, float R4, float R5, float jointsResult[7][3]) {
    if (this->robotType == ROBOT_TYPE::AXIS4) {
        R3 = 0;
        R4 = -(R1 + R2);
    }

    float a = cos(R0);
    float b = sin(R0);
    float c = this->geometry[0][0];
    float d = this->geometry[0][1];
    float e = this->geometry[0][2];
    float f = cos(R1);
    float g = sin(R1);
    float h = this->geometry[1][0];
    float i = this->geometry[1][1];
    float j = this->geometry[1][2];
    float k = cos(R2);
    float l = sin(R2);
    float m = this->geometry[2][0];
    float n = this->geometry[2][1];
    float o = this->geometry[2][2];
    float p = cos(R3);
    float q = sin(R3);
    float r = this->geometry[3][0];
    float s = this->geometry[3][1];
    float t = this->geometry[3][2];
    float u = cos(R4);
    float v = sin(R4);
    float w = this->geometry[4][0];
    float x = this->geometry[4][1];
    float y = this->geometry[4][2];
    float A = cos(R5 - PI);
    float B = sin(R5 - PI);


    // float C = 0 // this->geometry[5][0]
    // float D = 0 // this->geometry[5][1]
    // float E = 0 // this->geometry[5][2]

    jointsResult[0][0] = 0;
    jointsResult[0][1] = 0;
    jointsResult[0][2] = 0;

    jointsResult[1][0] = jointsResult[0][0] + a * c + b * e;
    jointsResult[1][1] = jointsResult[0][1] + d;
    jointsResult[1][2] = jointsResult[0][2] + -b * c + a * e;

    jointsResult[2][0] = jointsResult[1][0] + a * f * h - a * g * i + b * j;
    jointsResult[2][1] = jointsResult[1][1] + g * h + f * i;
    jointsResult[2][2] = jointsResult[1][2] - b * f * h + b * g * i + a * j;

    jointsResult[3][0] = jointsResult[2][0] + a * f * k * m - a * g * l * m - a * g * k * n - a * f * l * n + b * o;
    jointsResult[3][1] = jointsResult[2][1] + g * k * m + f * l * m + f * k * n - g * l * n;
    jointsResult[3][2] = jointsResult[2][2] - b * f * k * m + b * g * l * m + b * g * k * n + b * f * l * n + a * o;

    jointsResult[4][0] = jointsResult[3][0] + a * f * k * r - a * g * l * r - a * g * k * p * s - a * f * l * p * s + b * q * s + b * p *
                         t + a * g * k * q * t + a * f * l * q * t;
    jointsResult[4][1] = jointsResult[3][1] + g * k * r + f * l * r + f * k * p * s - g * l * p * s - f * k * q * t + g * l * q * t;
    jointsResult[4][2] = jointsResult[3][2] - b * f * k * r + b * g * l * r + b * g * k * p * s + b * f * l * p * s + a * q * s + a * p *
                         t - b * g * k * q * t - b * f * l * q * t;

    jointsResult[5][0] = jointsResult[4][0] + a * f * k * u * w - a * g * l * u * w - a * g * k * p * v * w - a * f * l * p * v * w + b *
                         q * v * w - a * g * k * p * u * x - a * f * l * p * u * x + b * q * u * x - a * f * k * v * x + a * g * l * v * x +
                         b * p * y + a * g *
                         k * q * y + a * f * l * q * y;
    jointsResult[5][1] = jointsResult[4][1] + g * k * u * w + f * l * u * w + f * k * p * v * w - g * l * p * v * w + f * k * p * u * x -
                         g * l * p * u * x - g * k * v * x - f * l * v * x - f * k * q * y + g * l * q * y;
    jointsResult[5][2] = jointsResult[4][2] - b * f * k * u * w + b * g * l * u * w + b * g * k * p * v * w + b * f * l * p * v * w + a *
                         q * v * w + b * g * k * p * u * x + b * f * l * p * u * x + a * q * u * x + b * f * k * v * x - b * g * l * v * x +
                         a * p * y - b * g *
                         k * q * y - b * f * l * q * y;

    float M[3][3] = {
        { a *g *k *p *u + a *f *l *p *u - b *q *u + a *f *k *v - a *g *l *v,
          +B *b *p - B *a *g *k *q + B *a *f *l *q + A *a *f *k *u - A *a *g *l *u - A *a *g *k *p *v - A *a *f *l *p *v + A *b *q *v,
          A *b *p + A *a *g *k *q + A *a *f *l *q - B *a *f *k *u + B *a *g *l *u + B *a *g *k *p *v + B *a *f *l *p *v -
          B *b *q *v                                                                                                                                                                                                                                                  },
        {                             -f * k * p * u + g * l * p * u + g * k * v + f * l * v,
                                      B * f * k * q + B * g * l * q + A * g * k * u + A * f * l * u + A * f * k * p * v - A * g * l * p * v,
                                      -A * f * k * q + A * g * l * q - B * g * k * u - B * f * l * u - B * f * k * p * v + B * g * l * p *
                                      v },
        { -b * g * k * p * u - b * f * l * p * u - a * q * u - b * f * k * v + b * g * l * v,
          +B * a * p - B * b * g * k * q - B * b * f * l * q - A * b * f * k * u + A * b * g * l * u + A * b * g * k * p * v + A * b * f *
          l * p * v + A * a * q * v,
          A * a * p - A * b * g * k * q - A * b * f * l * q + B * b * f * k * u - B * b * g * l * u - B * b * g * k * p * v - B * b * f *
          l * p * v - B * a * q *
          v                                                                                                                                                                                                                                                                                                                           },
    };

    // float M[3][3] = {
    //     { a *g *k *p *u + a *f *l *p *u - b *q *u + a *f *k *v - a *g *l *v,
    //       -B *b *p - B *a *g *k *q - B *a *f *l *q + A *a *f *k *u - A *a *g *l *u - A *a *g *k *p *v - A *a *f *l *p *v + A *b *q *v,
    //       A *b *p + A *a *g *k *q + A *a *f *l *q + B *a *f *k *u - B *a *g *l *u - B *a *g *k *p *v - B *a *f *l *p *v + B *b *q *v },
    //     {                             -f * k * p * u + g * l * p * u + g * k * v + f * l * v,
    //                                   B * f * k * q - B * g * l * q + A * g * k * u + A * f * l * u + A * f * k * p * v - A * g * l * p *
    // v,
    //                                   -A * f * k * q + A * g * l * q + B * g * k * u + B * f * l * u + B * f * k * p * v - B * g * l * p
    // *
    //                                   v },
    //     { -b * g * k * p * u - b * f * l * p * u - a * q * u - b * f * k * v + b * g * l * v,
    //       -B * a * p + B * b * g * k * q + B * b * f * l * q - A * b * f * k * u + A * b * g * l * u + A * b * g * k * p * v + A * b * f
    // *
    //       l * p * v + A * a * q * v,
    //       A * a * p - A * b * g * k * q - A * b * f * l * q - B * b * f * k * u + B * b * g * l * u + B * b * g * k * p * v + B * b * f *
    //       l * p * v + B * a * q * v },
    // };

    // http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf

    float theta = 0;
    float psi   = 0;
    float phi   = 0;

    if ((M[2][0] !=  1) || (M[2][0] !=  -1)) {
        theta = PI + asin(M[2][0]);
        psi   = atan2(M[2][1] / cos(theta), M[2][2] / cos(theta));
        phi   = atan2(M[1][0] / cos(theta), M[0][0] / cos(theta));
    } else {
        phi = 0; // anything; can set to

        if (M[2][0] ==  -1) {
            theta = PI / 2;
            psi   = phi + atan2(M[0][1], M[0][2]);
        } else {
            theta = -PI / 2;
            psi   = -phi + atan2(-M[0][1], -M[0][2]);
        }
    }

    jointsResult[6][0] = psi;
    jointsResult[6][1] = theta;
    jointsResult[6][2] = phi;

    if (this->debug) {
        log("+++++++++forward KINEMATICS++++++++++");
        log("J0 X " + String(jointsResult[0][0]) + " Y " + String(jointsResult[0][1]) + " Z " + String(jointsResult[0][2]) + "");
        log("J1 X " + String(jointsResult[1][0]) + " Y " + String(jointsResult[1][1]) + " Z " + String(jointsResult[1][2]) + "");
        log("J2 X " + String(jointsResult[2][0]) + " Y " + String(jointsResult[2][1]) + " Z " + String(jointsResult[2][2]) + "");
        log("J4 X " + String(jointsResult[4][0]) + " Y " + String(jointsResult[4][1]) + " Z " + String(jointsResult[4][2]) + "");
        log("J5 X " + String(jointsResult[5][0]) + " Y " + String(jointsResult[5][1]) + " Z " + String(jointsResult[5][2]) + "");
        log("J5 A " + String(jointsResult[6][0]) + " B " + String(jointsResult[6][1]) + " C " + String(jointsResult[6][2]) + "");
        log("---------forward KINEMATICS----------" + String(jointsResult[1][1]) + "");
    }
}

void Kinematic::cross(float vectorA[2], float  vectorB[2], float result[3]) {
    result[0] =  vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1];
    result[1] = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2];
    result[2] = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0];
}

float Kinematic::dot(float vectorA[3], float vectorB[3]) {
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2];
}

float Kinematic::angleBetween(float vectorA[2], float vectorB[2], float referenceVector[2]) {
    // angle = atan2(norm(cross(a, b)), dot(a, b))
    float cross[3];

    this->cross(vectorA, vectorB, cross);
    float norm = this->length3(cross);

    float angle =  atan2(norm, (vectorB[0] * vectorA[0] + vectorB[1] * vectorA[1] + vectorB[2] * vectorA[2]));


    float tmp = referenceVector[0] * vectorA[0] + referenceVector[1] * vectorA[1] + referenceVector[2] * vectorA[2];

    float sign = (tmp > 0) ? 1.0 : -1.0;

    return angle * sign;
}

float Kinematic::length3(float vector[2]) {
    return sqrt(pow(vector[0], 2) +  pow(vector[1], 2) +  pow(vector[2], 2));
}

float Kinematic::length2(float a, float b) {
    return sqrt(pow(a, 2) +  pow(b, 2));
}

void Kinematic::setDebug(bool debug) {
    this->debug = debug;
}
