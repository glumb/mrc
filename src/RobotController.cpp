#include "Kinematic.h"
#include "RobotController.h"
#include "Logger.h"


namespace {
Logger logger("RobotController");
}

#ifndef physicaltoLogicAngles
# define physicaltoLogicAngles(angles) { \
        (angles)[2] -= (angles)[1];      \
}
#endif // ifndef

// no kinematic coupling here because it must happen step by step in the servo update method to be able to move kincouping independent from
// speed
#define NORM(v, u, length) {                                                  \
        (length) = sqrt((u)[0] * (u)[0] + (u)[1] * (u)[1] + (u)[2] * (u)[2]); \
        (v)[0]   = (u)[0] / length;                                           \
        (v)[1]   = (u)[1] / length;                                           \
        (v)[2]   = (u)[2] / length;                                           \
}


#define ADDV(v, u, w) /* ADD Vector */ { \
        (v)[0] = (u)[0] + (w)[0];        \
        (v)[1] = (u)[1] + (w)[1];        \
        (v)[2] = (u)[2] + (w)[2];        \
}

#define SUBV(v, u, w) /* SUBtract Vector */ { \
        (v)[0] = (u)[0] - (w)[0];             \
        (v)[1] = (u)[1] - (w)[1];             \
        (v)[2] = (u)[2] - (w)[2];             \
}

#define MULVS(v, u, s) /* MULtiply Vector by */ { \
        (v)[0] = (u)[0] * s;                      \
        (v)[1] = (u)[1] * s;                      \
        (v)[2] = (u)[2] * s;                      \
}

#define CROSSVP(v, u, w) /* CROSS Vector Product */ { \
        (v)[0] = (u)[1] * (w)[2] - (u)[2] * (w)[1];   \
        (v)[1] = (u)[2] * (w)[0] - (u)[0] * (w)[2];   \
        (v)[2] = (u)[0] * (w)[1] - (u)[1] * (w)[0];   \
}

#define DOTVP(v, u) ((v)[0] * (u)[0] + (v)[1] * (u)[1] + (v)[2] * (u)[2])

#define NUMBER_OF_AXIS 6

RobotController::RobotController(MyServo *servos[], Kinematic *Kin, void(*logicToPhysicalAngles)(float[6])) {
    for (size_t i = 0; i < NUMBER_OF_AXIS; i++) {
        this->Servos[i] = servos[i];

        logicAngleLimits[i][0] = servos[i]->getMaxRadAngle();
        logicAngleLimits[i][1] = servos[i]->getMinRadAngle();
    }

    this->logicToPhysicalAngles = logicToPhysicalAngles;

    this->IK = Kin;

    this->getCurrentPose(this->targetPose);

    this->movementMethod = LINEAR;
    this->state          = IDLE;

    this->interpolationDistanceIncrement         = 1;
    this->interpolationOrientationAngleIncrement = 5;

    this->maxVelocity = 70; // in units per s

    this->moveAsFarAsPossibleOnOutOfBound = false;

    this->getCurrentLogicAngles(this->targetAngleBuffer);

    this->setTargetAngles(this->targetAngleBuffer);

    this->process();
}

void RobotController::startTransaction() {
    this->inTransaction = true;
}

void RobotController::endTransaction() {
    this->inTransaction = false;
}

void RobotController::setMaxVelocity(float velocity) {
    this->maxVelocity = velocity;
}

float RobotController::getMaxVelocity() {
    return this->maxVelocity;
}

void RobotController::resetPose() {
    float angles[NUMBER_OF_AXIS] = { 0 };

    this->setTargetAngles(angles);
}

void RobotController::moveToMinPose() {
    float angles[NUMBER_OF_AXIS];

    for (size_t i = 0; i < NUMBER_OF_AXIS; i++) {
        // add a little to not conflict with limits todo
        angles[i] = this->Servos[i]->getMinRadAngle() + 5.0 / 180.0 * PI;
    }

    this->setTargetAngles(angles);
}

void RobotController::moveToMaxPose() {
    float angles[NUMBER_OF_AXIS];

    for (size_t i = 0; i < NUMBER_OF_AXIS; i++) {
        // sub a little to not conflict with limits todo
        angles[i] = this->Servos[i]->getMaxRadAngle() - 5.0 / 180.0 * PI;
    }

    this->setTargetAngles(angles);
}

void RobotController::setMovementMethod(MOVEMENT_METHODS method) {
    this->movementMethod = method;
}

RobotController::MOVEMENT_METHODS RobotController::getMovementMethod() {
    return this->movementMethod;
}

void RobotController::getTargetPose(float targetPose[6]) {
    targetPose = this->targetPose; // todo
}

void RobotController::setTargetPose(float x,
                                    float y,
                                    float z,
                                    float a,
                                    float b,
                                    float c) {
    this->_setTargetPose(x, y, z, a, b, c);
}

void RobotController::setTargetPose(POSITION position, float value) {
    switch (position) {
    case X:
        this->targetPoseBuffer[0] = value;
        break;

    case Y:
        this->targetPoseBuffer[1] = value;
        break;

    case Z:
        this->targetPoseBuffer[2] = value;
        break;

    case A:
        this->targetPoseBuffer[3] = value;
        break;

    case B:
        this->targetPoseBuffer[4] = value;
        break;

    case C:
        this->targetPoseBuffer[5] = value;
        break;
    }

    this->state = PREPARE_MOVE;
}

void RobotController::getCurrentLogicAngles(float currentAngles[]) {
    for (unsigned int i = 0; i < NUMBER_OF_AXIS; i++) {
        currentAngles[i] = this->Servos[i]->getCurrentAngle();
    }
}

// todo add stop method() targetPose = currentPose

void RobotController::_setTargetPose(float x, float y, float z, float a, float b, float c) {
    // todo constrain the target pose here. E.g 4 axis robot B=PI C=0...
    this->targetPoseBuffer[0] = x;
    this->targetPoseBuffer[1] = y;
    this->targetPoseBuffer[2] = z;
    this->targetPoseBuffer[3] = a;
    this->targetPoseBuffer[4] = b;
    this->targetPoseBuffer[5] = c;

    // set the target angles to be used by setTargetAngle()
    // angles may not have been set to the servos when setting multiple target angles

    this->state = PREPARE_MOVE;
}

void RobotController::setTargetAngles(float targetAngles[6]) {
    // todo constrain the target angles here. E.g 4 axis robot A4 = -(A1+A2)

    float TCP[6];

    // complicated way, since interpolation method may still be linear, so we need the target pos anyways
    this->IK->forward(
        targetAngles[0],
        targetAngles[1],
        targetAngles[2],
        targetAngles[3],
        targetAngles[4],
        targetAngles[5],
        TCP
        );

    logger.info("--still here--");
    logger.info(targetAngles[0]);
    logger.info(targetAngles[1]);
    logger.info(targetAngles[2]);
    logger.info(targetAngles[3]);
    logger.info(targetAngles[4]);
    logger.info(targetAngles[5]);

    this->_setTargetPose(
        TCP[0],
        TCP[1],
        TCP[2],
        TCP[3],
        TCP[4],
        TCP[5]);
}

void RobotController::setTargetAngle(unsigned int index,
                                     float        targetAngle) {
    // float angles[NUMBER_OF_AXIS];

    // setting multiple angles in series ovverides the angles since they are not yet set to the servos
    // this->getTargetAngles(angles);
    // angles[index] = targetAngle;
    // this->setTargetAngles(angles);

    this->targetAngleBuffer[index] = targetAngle;
    this->setTargetAngles(this->targetAngleBuffer);

    // Serial.println(this->targetPose[0]);
    // todo
    // this->Servos[index]->setTargetRadAngle(targetAngle);
}

void RobotController::_applyTimedTargetAngles(float targetAngles[6], float targetTime) {
    float maxTime = targetTime;

    // todo take into account that the target angle may be out of angle and thus be far away
    for (unsigned int i = 0; i < NUMBER_OF_AXIS; i++) {
        float dAngle = abs(targetAngles[i] - this->Servos[i]->getCurrentAngle());
        float dTime  = dAngle / this->Servos[i]->getMaxAngleVelocity();

        if (dTime > maxTime) maxTime = dTime;
    }

    if (maxTime > targetTime) {
        logger.warning("could not move in time: " + String(targetTime));
        logger.warning(" using instead time: " + String(maxTime));
    }

    // Serial.println(targetAngles[5]);
    if (maxTime != 0) {
        for (unsigned int j = 0; j < NUMBER_OF_AXIS; j++) {
            float dAngle = abs(targetAngles[j] - this->Servos[j]->getCurrentAngle());
            this->Servos[j]->setCurrentAngleVelocity(dAngle / maxTime);
            this->Servos[j]->setTargetRadAngle(targetAngles[j]);
        }
    }
}

void RobotController::getTargetAngles(float targetAngles[]) {
    for (unsigned int i = 0; i < NUMBER_OF_AXIS; i++) {
        targetAngles[i] = this->Servos[i]->getTargetRadAngle();
    }
}

float RobotController::getTargetAngle(unsigned int index) {
    if (index >= NUMBER_OF_AXIS) {
        Serial.print("WARING, can not getTargetAngle(), out of index");
        return 0;
    }
    return this->Servos[index]->getTargetRadAngle();
}

void RobotController::getCurrentPose(float Pose[6]) {
    float angles[6];

    for (size_t i = 0; i < 6; i++) {
        angles[i] = this->Servos[i]->getCurrentAngle();
    }
    physicaltoLogicAngles(angles);
    this->IK->forward(
        angles[0],
        angles[1],
        angles[2],
        angles[3],
        angles[4],
        angles[5],
        Pose
        );
}

bool RobotController::isMoving() {
    return this->state != IDLE;
}

bool RobotController::atTargetPose() {
    return this->PoseEquals(this->startPose, this->targetPose);
}

bool RobotController::PoseEquals(float pos1[6], float pos2[6]) {
    for (unsigned int i = 0; i < 6; i++) {
        if (pos1[i] != pos2[i]) {
            return false;
        }
    }
    return true;
}

void RobotController::setLogicAngleLimits(float angleLimitsRad[6][2]) {
    for (size_t i = 0; i < 6; i++) {
        this->logicAngleLimits[i][0] = angleLimitsRad[i][0];
        this->logicAngleLimits[i][1] = angleLimitsRad[i][1];
    }
}

void RobotController::getCurrentPhysicalAngles(float angles[6]) {

    getCurrentLogicAngles(angles);
    this->logicToPhysicalAngles(angles);
}

void RobotController::process() {
    static unsigned int currentInterpolationStep = 0;
    static unsigned int totalInterpolationSteps  = 0;

    switch (this->state) {
    case IDLE:
        break;

    case MOVING:
    {
        logger.info("MOVING");

        // may not be the target pose though
        bool atTargetAngle = true;

        // todo use a timer in set angles to determine if on target, instead of querying every servo
        for (unsigned int i = 0; i < 6; i++) {
            if (!this->Servos[i]->atTargetAngle()) {
                atTargetAngle = false;
                logger.info("not at Target angle" + String(i));
            }
        }

        // for (size_t i = 0; i < 6; i++) { todo stop all on out of range?
        //     if (servos[i]->getOutOfRange()) {
        //
        //     }
        // }

        if (atTargetAngle) {
            logger.info("at Target angle");
            logger.info(currentInterpolationStep);
            logger.info(totalInterpolationSteps);

            if (currentInterpolationStep == totalInterpolationSteps) {
                this->state = IDLE;
            } else {
                this->state = START_MOVE;
            }
        }
        break;
    }

    case PREPARE_MOVE: {
        logger.info("PREPARE_MOVE");

        // transactions are used to set multiple values (x,y,r1,r2,r3..) in one set and apply them all at once
        // do not strat a new move, when in a transaction
        if (this->inTransaction) {
            break;
        }

        if (this->PoseEquals(this->targetPose, this->targetPoseBuffer)) {
            this->state = IDLE;
            break;
        }

        this->IK->inverse(this->targetPoseBuffer[0],
                          this->targetPoseBuffer[1],
                          this->targetPoseBuffer[2],
                          this->targetPoseBuffer[3],
                          this->targetPoseBuffer[4],
                          this->targetPoseBuffer[5],
                          this->targetAngleBuffer);

        // delay(5000);
        // Serial.println(this->targetAngleBuffer[0]);
        // Serial.println(this->targetAngleBuffer[1]);
        // Serial.println(this->targetAngleBuffer[2]);
        // Serial.println(this->targetAngleBuffer[3]);
        // Serial.println(this->targetAngleBuffer[4]);
        // Serial.println(this->targetAngleBuffer[5]);
        // delay(25000);
        // Serial.println(this->targetPoseBuffer[0]);
        // Serial.println(this->targetPoseBuffer[1]);
        // Serial.println(this->targetPoseBuffer[2]);
        // Serial.println(this->targetPoseBuffer[3]);
        // Serial.println(this->targetPoseBuffer[4]);
        // Serial.println(this->targetPoseBuffer[5]);
        // memcpy(this->startAngles,  this->targetAngleBuffer,       6 * sizeof(float)); // todo use current pose

        this->getCurrentPose(this->startPose); // todo use current pose
        // memcpy(this->startPose,  this->targetPose,       6 * sizeof(float)); // todo use current pose
        memcpy(this->targetPose, this->targetPoseBuffer, 6 * sizeof(float));

        currentInterpolationStep = 0;

        switch (this->movementMethod) {
        case P2P:
            totalInterpolationSteps = 0;
            break;

        case LINEAR:


            // rotate a (1,0,0) Vector according to ZYX Euler angle rotation

            float targetOrientationVector[3];

            float cb = cos(this->targetPose[4]);
            float sb = sin(this->targetPose[4]);
            float cc = cos(this->targetPose[5]);
            float sc = sin(this->targetPose[5]);

            targetOrientationVector[0] = cb * cc;
            targetOrientationVector[1] = cb * sc;
            targetOrientationVector[2] = -sb;

            cb = cos(this->startPose[4]);
            sb = sin(this->startPose[4]);
            cc = cos(this->startPose[5]);
            sc = sin(this->startPose[5]);

            this->startOrientationVector[0] = cb * cc;
            this->startOrientationVector[1] = cb * sc;
            this->startOrientationVector[2] = -sb;


            // init the rotation Axis rodrigues rotation
            double length;

            NORM(this->targetOrientationVectorNorm, targetOrientationVector, length);

            CROSSVP(this->rotationAxisVectorNorm, this->startOrientationVector, this->targetOrientationVectorNorm); // this->rotationAxisVectorNorm
            NORM(this->rotationAxisVectorNorm,     this->rotationAxisVectorNorm, length);
            NORM(this->startOrientationVectorNorm, this->startOrientationVector, length);

            this->interpolationRotationAngle = acos(DOTVP(this->startOrientationVectorNorm, this->targetOrientationVectorNorm));

            // set the interpolation steps based on orientation angle and distance
            float dx = this->targetPose[0] - this->startPose[0];
            float dy = this->targetPose[1] - this->startPose[1];
            float dz = this->targetPose[2] - this->startPose[2];


            // todo use this->interpolationDistanceIncrement to determine steps
            //             // todo include only rotation
            float distance =  sqrt(pow(dx, 2) +  pow(dy, 2) +  pow(dz, 2));

            int distanceSteps = distance / this->interpolationDistanceIncrement;
            int angleSteps    = this->interpolationRotationAngle / this->interpolationOrientationAngleIncrement;

            // todo use (distanceSteps>angleSteps)?distanceSteps:angleSteps
            totalInterpolationSteps = 20;
            break;
        }


        this->state = START_MOVE;

        this->process();
        break;
    }

    case START_MOVE:
    {
        logger.info("START_MOVE");
        float tmpTargetPose[6] = { 0 };
        float dTime            = 0;

        switch (this->movementMethod) {
        case P2P:
            logger.info("P2P");
            memcpy(tmpTargetPose, this->targetPose, 6 * sizeof(float)); // todo do we need memcpy?
            dTime = this->maxVelocity;                                  // todo this is basically using v as the time
            break;

        case LINEAR:

            // increment first because STATE changes on currentInterpolationStep==totalInterpolationSteps and interpolation step 0 is the
            // current position
            currentInterpolationStep++;

            float fraction;

            if (totalInterpolationSteps == 0) {
                fraction = 1;
            } else {
                fraction = (float)currentInterpolationStep / (float)totalInterpolationSteps;
            }

            float dx = this->targetPose[0] - this->startPose[0];

            // logger.info("this->targetPose[0] " + String(this->targetPose[0]), false);
            // logger.info(dx,                                                   false);
            float dy = this->targetPose[1] - this->startPose[1];

            // logger.info("this->targetPose[1] " + String(this->targetPose[1]), false);
            // logger.info(dy,                                                   false);
            float dz = this->targetPose[2] - this->startPose[2];

            // logger.info("this->targetPose[2] " + String(this->targetPose[2]), false);
            // logger.info(dz,                                                   false);
            float da = this->targetPose[3] - this->startPose[3];

            // logger.info("this->targetPose[3] " + String(this->targetPose[3]), false);
            // logger.info(da,                                                   false);
            float db = this->targetPose[4] - this->startPose[4];

            // logger.info("this->targetPose[4] " + String(this->targetPose[4]), false);
            // logger.info(db,                                                   false);
            float dc = this->targetPose[5] - this->startPose[5];

            // logger.info("this->targetPose[5] " + String(this->targetPose[5]), false);
            // logger.info(dc,                                                   false);

            // todo do this properly with quaternions

            float b = 0;
            float c = 0;

            if ((abs(db) > 0.0001) || (abs(dc) > 0.0001)) {
                float targetVector[3];

                this->rodrigues(targetVector, this->rotationAxisVectorNorm, this->startOrientationVector,
                                fraction * this->interpolationRotationAngle);

                float rotationMatrix[3][3];

                float length;
                NORM(targetVector, targetVector, length);

                // X component
                rotationMatrix[0][0] = targetVector[0];
                rotationMatrix[1][0] = targetVector[1];
                rotationMatrix[2][0] = targetVector[2];

                // todo simplify


                if ((rotationMatrix[2][0] !=  1) || (rotationMatrix[2][0] !=  -1)) {
                    b = PI + asin(rotationMatrix[2][0]);
                    c = atan2(rotationMatrix[1][0] / cos(b), rotationMatrix[0][0] / cos(b));
                } else {
                    c = 0; // anything; can set to

                    if (rotationMatrix[2][0] ==  -1) {
                        b = PI / 2;
                    } else {
                        b = -PI / 2;
                    }
                }
            } else {
                b = this->targetPose[4];
                c = this->targetPose[5];
            }

            logger.info("fraction " + String(fraction));

            float distance =  sqrt(pow(dx, 2) +  pow(dy, 2) +  pow(dz, 2)); // todo if distance == 0, only pose change

            dx = dx * fraction;
            dy = dy * fraction;
            dz = dz * fraction;

            da = da * fraction;

            // Serial.println(da);
            // delay(100);
            // Serial.println(this->targetPose[3]);
            // delay(100);
            // Serial.println(this->startPose[3]);
            // delay(100);
            // Serial.println("--end--");
            // delay(100);

            tmpTargetPose[0] = this->startPose[0] + dx;
            tmpTargetPose[1] = this->startPose[1] + dy;
            tmpTargetPose[2] = this->startPose[2] + dz;

            tmpTargetPose[3] = this->startPose[3] + da;
            tmpTargetPose[4] = b;
            tmpTargetPose[5] = c;

            logger.info("--- A B C ---",  false);
            logger.info(tmpTargetPose[3], false);
            logger.info(tmpTargetPose[4], false);
            logger.info(tmpTargetPose[5], false);
            logger.info("-----",          false);

            // v = s/t  t = s/v
            dTime = (distance / (float)totalInterpolationSteps) / this->maxVelocity;

            break;

            //
            // case CIRCULAR:
            //     break;
        }


        float targetAngles[NUMBER_OF_AXIS] = { 0 }; // todo check if tmpTargetPose angles >90° for new Pose - not linear anymore
        unsigned int returnCode            = this->IK->inverse(
            tmpTargetPose[0],
            tmpTargetPose[1],
            tmpTargetPose[2],
            tmpTargetPose[3],
            tmpTargetPose[4],
            tmpTargetPose[5],
            targetAngles
            );

        // todo set current angle to last angle if out of angle... or set to current servo position
        bool outOfAngle = false;

        // Serial.println(dTime*1000);

        if (returnCode == Kinematic::OK) {
            // Serial.println("--- A B C ---");
            // Serial.println(targetAngles[0]);
            // Serial.println(targetAngles[1]);
            // Serial.println(targetAngles[2]);
            // Serial.println(targetAngles[3]);
            // Serial.println(targetAngles[4]);
            // Serial.println(targetAngles[5]);
            // Serial.println("-----");
            //
            // targetAngles[2] += targetAngles[1];

            for (size_t i = 0; i < NUMBER_OF_AXIS; i++) { // todo remove check here. change notation of logic angles
                // todo pass custom checking function to comprise 4 Axis kinematic coupled robots
                if ((targetAngles[i] < this->logicAngleLimits[i][0]) || (targetAngles[i] > this->logicAngleLimits[i][1])) {
                    logger.warning("servo " + String(i) + " is out of logic angle: " + String(targetAngles[i] / PI * 180) + " min: "
                                   + String(this->logicAngleLimits[i][0] / PI * 180) + " max "
                                   + String(this->logicAngleLimits[i][1] / PI * 180));


                    outOfAngle = true;
                }
            }

            this->logicToPhysicalAngles(targetAngles);


            for (size_t i = 0; i < NUMBER_OF_AXIS; i++) { // todo remove check here. change notation of logic angles
                // todo pass custom checking function to comprise 4 Axis kinematic coupled robots
                if ((targetAngles[i] < this->Servos[i]->getMinRadAngle()) || (targetAngles[i] > this->Servos[i]->getMaxRadAngle())) {
                    logger.warning("servo " + String(i) + " is out of physical angle: " + String(targetAngles[i] / PI * 180) + " min: "
                                   + String(this->Servos[i]->getMinRadAngle() / PI * 180) + " max "
                                   + String(this->Servos[i]->getMaxRadAngle() / PI * 180));


                    outOfAngle = true;
                }
            }

            // todo handle singularity
            if (abs(targetAngles[4] - (PI / 2.0)) < 0.05) { // axis 5 and 3 in line
                Serial.println("singularity axis 5,3");

                targetAngles[3] = this->Servos[3]->getCurrentAngle();
                targetAngles[5] = this->Servos[5]->getCurrentAngle();
            }

            if (!outOfAngle) {
                // Serial.println("A5 "+String(targetAngles[5]/PI*180));
                this->_applyTimedTargetAngles(targetAngles, dTime);
                this->state = MOVING;
            } else if (this->moveAsFarAsPossibleOnOutOfBound) { // todo rename outOfAngle
                // Serial.println("A5 "+String(targetAngles[5]/PI*180));
                this->_applyTimedTargetAngles(targetAngles, dTime);
                this->state = MOVING;
            } else {
                this->state = IDLE;
            }
        } else {
            logger.warning("Oh noo, out of bounds");
            this->state = IDLE;
        }

        break;
    }
    }
}

void RobotController::rodrigues(float ret[3], float unit[3], float v[3], float angleRad)
{
    float ret1[3] = { 0 };
    float ret2[3] = { 0 };
    float ret3[3] = { 0 };

    MULVS(ret1, v, cos(angleRad));
    CROSSVP(ret2, unit, v);
    MULVS(ret2, ret2, sin(angleRad));

    MULVS(ret3, unit, DOTVP(unit, v));
    MULVS(ret3, ret3, (1 - cos(angleRad)));

    // v*cos(angleRad)+cross(unit,v)*sin(angleRad)+unit*dot(unit,v)*(1-cos(angleRad))

    // ret1 + ret2 + ret3

    ADDV(ret1, ret1, ret2);
    ADDV(ret1, ret1, ret3);

    ret[0] = ret1[0];
    ret[1] = ret1[1];
    ret[2] = ret1[2];
}
