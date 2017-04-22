#ifndef KINEMATIC_H
#define KINEMATIC_H


class Kinematic {
public:

    enum ROBOT_TYPE { AXIS6, AXIS4 };

    explicit Kinematic(float geometry[5][3]);

    static const unsigned int OK;
    static const unsigned int OUT_OF_RANGE;


    int inverse(float x,
                float y,
                float z,
                float a,
                float b,
                float c,
                float angles[6]);
    void forward(float A0,
                 float A1,
                 float A2,
                 float A3,
                 float A4,
                 float A5,
                 float jointsResult[6]);

    void calculateCoordinates(float A0,
                              float A1,
                              float A2,
                              float A3,
                              float A4,
                              float A5,
                              float jointsResult[6][3]);
    void setDebug(bool debug);

private:

    float length2(float a,
                  float b);
    float length3(float vector[2]);
    float angleBetween(float vectorA[2],
                       float vectorB[2],
                       float referenceVector[2]);
    float dot(float vectorA[3],
              float vectorB[3]);
    void  cross(float vectorA[2],
                float vectorB[2],
                float result[3]);


    bool debug;
    float V1_length_x_z;
    float V4_length_x_y_z;
    float geometry[5][3];
    float J_initial_absolute[5][3];
    float A_corrected[6];
};
#endif // ifndef KINEMATIC_H
