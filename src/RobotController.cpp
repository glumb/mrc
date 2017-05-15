#include "Kinematic.h"
#include "RobotController.h"
#include "Logger.h"


namespace {
Logger logger("RobotController");
}


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

RobotController::RobotController(MyServo   *servos[],
                                 Kinematic& _Kinematic,
                                 float      lal[6][2],
                                 void(*_logicalToPhysicalAngles)(float[6]),
                                 void(*_physicalToLogicalAngles)(float[6])) :
    Servos{servos[0], servos[1], servos[2], servos[3], servos[4], servos[5]},
    _Kinematic(_Kinematic),
    logicalAngleLimits{{
        lal[0][0], lal[0][1]
    }, { lal[1][0], lal[1][1] }, { lal[2][0], lal[2][1] }, { lal[3][0], lal[3][1] }, { lal[4][0], lal[4][1] }, { lal[5][0], lal[5][1] }},
    logicalToPhysicalAngles(_logicalToPhysicalAngles),
    physicalToLogicalAngles(_physicalToLogicalAngles)
{
    this->movementMethod = MOVEMENT_METHODS::P2P;
    this->state          = IDLE;

    this->interpolationDistanceIncrement         = 0.5;
    this->interpolationOrientationAngleIncrement = 15;

    this->maxVelocity = 70; // in units per s

    this->moveAsFarAsPossibleOnOutOfBound = false;

    this->getCurrentPose(this->targetPose);
    this->getCurrentLogicalAngles(this->targetAngles);

    // this->process();
}

void RobotController::startTransaction() {
    this->inTransaction = true;
}

void RobotController::endTransaction() {
    this->inTransaction = false;
}

void RobotController::stop() {
    this->state = STATES::IDLE;
}

void RobotController::setMaxVelocity(float velocity) {
    this->maxVelocity = velocity;
}

float RobotController::getMaxVelocity() {
    return this->maxVelocity;
}

void RobotController::setMovementMethod(MOVEMENT_METHODS method) {
    this->movementMethod = method;
}

RobotController::MOVEMENT_METHODS RobotController::getMovementMethod() {
    return this->movementMethod;
}

void RobotController::getCurrentPose(float pose[6]) {
    float angles[6];

    for (size_t i = 0; i < 6; i++) {
        angles[i] = this->Servos[i]->getCurrentAngle();

        // std::cout << "get Pose angles "<<angles[i] << '\n';
    }
    this->physicalToLogicalAngles(angles);

    this->_Kinematic.forward(
        angles[0],
        angles[1],
        angles[2],
        angles[3],
        angles[4],
        angles[5],
        pose
        );
}

float RobotController::getCurrentPose(POSITION position) {
    float pose[6];

    this->getCurrentPose(pose);

    switch (position) {
    case X:
        return pose[0];

        break;

    case Y:
        return pose[1];

        break;

    case Z:
        return pose[2];

        break;

    case A:
        return pose[3];

        break;

    case B:
        return pose[4];

        break;

    case C:
        return pose[5];

        break;
    }
}

void RobotController::setTargetPose(float x,
                                    float y,
                                    float z,
                                    float a,
                                    float b,
                                    float c) {
    // todo constrain the target pose here. E.g 4 axis robot B=PI C=0...
    this->targetPose[0] = x;
    this->targetPose[1] = y;
    this->targetPose[2] = z;
    this->targetPose[3] = a;
    this->targetPose[4] = b;
    this->targetPose[5] = c;

    this->targetPoseChanged = true;

    // set the target angles to be used by setTargetAngle()
    // angles may not have been set to the servos when setting multiple target angles

    this->state = PREPARE_MOVE;
}

void RobotController::setTargetPose(float pose[6]) {
    // todo constrain the target pose here. E.g 4 axis robot B=PI C=0...
    this->targetPose[0] = pose[0];
    this->targetPose[1] = pose[1];
    this->targetPose[2] = pose[2];
    this->targetPose[3] = pose[3];
    this->targetPose[4] = pose[4];
    this->targetPose[5] = pose[5];

    this->targetPoseChanged = true;

    // set the target angles to be used by setTargetAngle()
    // angles may not have been set to the servos when setting multiple target angles

    this->state = PREPARE_MOVE;
}

void RobotController::setTargetPose(POSITION position, float value) {
    switch (position) {
    case X:
        this->targetPose[0] = value;
        break;

    case Y:
        this->targetPose[1] = value;
        break;

    case Z:
        this->targetPose[2] = value;
        break;

    case A:
        this->targetPose[3] = value;
        break;

    case B:
        this->targetPose[4] = value;
        break;

    case C:
        this->targetPose[5] = value;
        break;
    }
    this->targetPoseChanged = true;

    this->state = PREPARE_MOVE;
}

void RobotController::getTargetPose(float targetPose[6]) {
    targetPose[0] = this->targetPose[0];
    targetPose[1] = this->targetPose[1];
    targetPose[2] = this->targetPose[2];
    targetPose[3] = this->targetPose[3];
    targetPose[4] = this->targetPose[4];
    targetPose[5] = this->targetPose[5];
}

float RobotController::getTargetPose(POSITION position) {
    switch (position) {
    case X:
        return this->targetPose[0];

        break;

    case Y:
        return this->targetPose[1];

        break;

    case Z:
        return this->targetPose[2];

        break;

    case A:
        return this->targetPose[3];

        break;

    case B:
        return this->targetPose[4];

        break;

    case C:
        return this->targetPose[5];

        break;
    }
}

void RobotController::getCurrentLogicalAngles(float currentAngles[6]) {
    for (unsigned int i = 0; i < 6; i++) {
        currentAngles[i] = this->Servos[i]->getCurrentAngle();
    }
    this->physicalToLogicalAngles(currentAngles);
}

float RobotController::getCurrentLogicalAngle(unsigned int index) {
    float angles[6];

    this->getCurrentLogicalAngles(angles);
    return angles[index];
}

void RobotController::setTargetLogicalAngles(float targetAngles[6]) {
    // todo constrain the target angles here. E.g 4 axis robot A4 = -(A1+A2)

    this->targetAngles[0] = targetAngles[0];
    this->targetAngles[1] = targetAngles[1];
    this->targetAngles[2] = targetAngles[2];
    this->targetAngles[3] = targetAngles[3];
    this->targetAngles[4] = targetAngles[4];
    this->targetAngles[5] = targetAngles[5];

    this->targetAnglesChanged = true;

    this->state = PREPARE_MOVE;
}

void RobotController::setTargetLogicalAngle(unsigned int index,
                                            float        targetAngle) {
    this->targetAngles[index] = targetAngle;
    this->targetAnglesChanged = true;

    this->state = PREPARE_MOVE;
}

void RobotController::getTargetLogicalAngles(float targetAngles[]) {
    for (unsigned int i = 0; i < 6; i++) {
        targetAngles[i] = this->targetAngles[i];
    }
}

float RobotController::getTargetLogicalAngle(unsigned int index) {
    if (index >= 6) {
        Serial.print("WARING, can not getTargetAngle(), out of index");
        return 0;
    }
    return this->targetAngles[index];
}

void RobotController::getCurrentPhysicalAngles(float angles[6]) {
    this->getCurrentLogicalAngles(angles);
    this->logicalToPhysicalAngles(angles);
}

float RobotController::getCurrentPhysicalAngle(unsigned int index) {
    float angles[6];

    this->getCurrentLogicalAngles(angles);
    this->logicalToPhysicalAngles(angles);
    return angles[index];
}

void RobotController::getTargetPhysicalAngles(float targetAngles[6]) {
    this->getTargetLogicalAngles(targetAngles);
    this->logicalToPhysicalAngles(targetAngles);
}

float RobotController::getTargetPhysicalAngle(unsigned int index) {
    float angles[6];

    this->getTargetLogicalAngles(angles);
    this->logicalToPhysicalAngles(angles);

    return angles[index];
}

bool RobotController::isMoving() {
    return this->state != IDLE;
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

        if (atTargetAngle) {
            // std::cout << "/* attargetangelell */" << '\n';
            logger.info("at Target angle");
            logger.info(currentInterpolationStep);
            logger.info(totalInterpolationSteps);

            // std::cout << "-0=====================interpol====== "<<currentInterpolationStep<<" total: "<<totalInterpolationSteps << '\n';
            if (currentInterpolationStep == totalInterpolationSteps) {
                this->state = IDLE;
            } else {
                this->state = START_MOVE;

                this->process();

                // todo process again to remove lag?
            }
        }
        break;
    }

    case PREPARE_MOVE: {
        logger.info("PREPARE_MOVE");

        // transactions are used to set multiple values (x,y,r1,r2,r3..) in one set and apply them all at once
        // do not strat a new move, when in a transaction
        if (this->inTransaction) {
            logger.info("in transaction");
            break;
        }

        // why two handling changes:
        //  changes to angles result in deterministic pose changes
        //  changes to pose however may lead to different angles based on configuration (still kind of deterministic, though)
        //  moving angles across configuration boundries may lead to inexpected behaviour. Therefore changing only angles should not involve
        // _Kinematic for movement.
        //
        // set Angles and pose to be in sync
        if (this->targetAnglesChanged) {
            logger.info("target angles changed");
            this->_Kinematic.forward(this->targetAngles[0],
                                     this->targetAngles[1],
                                     this->targetAngles[2],
                                     this->targetAngles[3],
                                     this->targetAngles[4],
                                     this->targetAngles[5],
                                     this->targetPose);
        } else if (this->targetPoseChanged) {
            logger.info("target pose changed");
            this->_Kinematic.inverse(this->targetPose[0],
                                     this->targetPose[1],
                                     this->targetPose[2],
                                     this->targetPose[3],
                                     this->targetPose[4],
                                     this->targetPose[5],
                                     this->targetAngles);
        } else {
            logger.error("nothing changed, still PREPARE MOVE");

            // pose not changed
            this->state = IDLE;
        }

        if (this->targetPoseChanged && this->targetAnglesChanged) {
            logger.error("Do not change pose and angles in one transaction!");
            this->targetAnglesChanged = false;
            this->targetPoseChanged   = false;
            this->state = IDLE;
            return;
        }

        this->targetAnglesChanged = false;
        this->targetPoseChanged   = false;


        this->getCurrentPose(this->startPose);            // todo use current pose
        this->getCurrentLogicalAngles(this->startAngles); // todo use current pose

        logger.info("start pose 0: " + String(this->startPose[0]));
        logger.info("start pose 1: " + String(this->startPose[1]));
        logger.info("start pose 2: " + String(this->startPose[2]));
        logger.info("start pose 3: " + String(this->startPose[3]));
        logger.info("start pose 4: " + String(this->startPose[4]));
        logger.info("start pose 5: " + String(this->startPose[5]));

        logger.info("start angles 0: " + String(this->startAngles[0] * RAD_TO_DEG));
        logger.info("start angles 1: " + String(this->startAngles[1] * RAD_TO_DEG));
        logger.info("start angles 2: " + String(this->startAngles[2] * RAD_TO_DEG));
        logger.info("start angles 3: " + String(this->startAngles[3] * RAD_TO_DEG));
        logger.info("start angles 4: " + String(this->startAngles[4] * RAD_TO_DEG));
        logger.info("start angles 5: " + String(this->startAngles[5] * RAD_TO_DEG));

        if (this->_anglesEqual(this->startAngles, this->targetAngles)) {
            this->state = IDLE;
            break;
        }

        // setup move
        currentInterpolationStep = 0;

        switch (this->movementMethod) {
        case P2P:
            totalInterpolationSteps = 1;
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

            int distanceSteps = (distance / this->interpolationDistanceIncrement) + 1;                                 // |-.-.-.-.-.| 5
                                                                                                                       // units '-' is 5
                                                                                                                       // steps '.'
            int angleSteps    = (this->interpolationRotationAngle / this->interpolationOrientationAngleIncrement) + 1; // +1 to not divide
                                                                                                                       // by 0 later

            totalInterpolationSteps = (distanceSteps > angleSteps) ? distanceSteps : angleSteps;

            // std::cout << "totalInterpolationSteps " << totalInterpolationSteps << '\n';

            break;
        }


        this->state = START_MOVE;

        this->process();
        break;
    }

    case START_MOVE:
    {
        logger.info("START_MOVE");
        float tmpTargetAngles[6];
        float moveInTime;

        // increment first to start with step 1
        currentInterpolationStep++;

        switch (this->movementMethod) {
        case P2P:
            logger.info("P2P");
            moveInTime = this->maxVelocity; // todo time is basically velocity
            memcpy(tmpTargetAngles, this->targetAngles, 6 * sizeof(float));
            break;

        case LINEAR:
        {
            logger.info("LINEAR");
            float fraction;

            // |-.-.-.-.-| '.':steps '-':sections
            //     ^ step: 2, fraction: 2/5 totalStepCount: 5
            fraction = (float)(currentInterpolationStep) / (float)(totalInterpolationSteps);

            float dx = this->targetPose[0] - this->startPose[0];
            float dy = this->targetPose[1] - this->startPose[1];
            float dz = this->targetPose[2] - this->startPose[2];
            float da = this->targetPose[3] - this->startPose[3];
            float db = this->targetPose[4] - this->startPose[4];
            float dc = this->targetPose[5] - this->startPose[5];

            logger.info("--- dX dY dZ dA dB dC ---");
            logger.info(dx);
            logger.info(dy);
            logger.info(dz);
            logger.info(da);
            logger.info(db);
            logger.info(dc);
            logger.info("-----");

            // todo do this properly with quaternions

            float a = this->targetPose[3];
            float b = this->targetPose[4];
            float c = this->targetPose[5];


            logger.info("fraction " + String(fraction));
            logger.info("currentInterpolationStep " + String(currentInterpolationStep));
            logger.info("totalInterpolationSteps " + String(totalInterpolationSteps));

            float distance =  sqrt(pow(dx, 2) +  pow(dy, 2) +  pow(dz, 2)); // todo if distance == 0, only pose change

            dx = dx * fraction;
            dy = dy * fraction;
            dz = dz * fraction;

            da = da * fraction;

            float tmpTargetPose[6];

            tmpTargetPose[0] = this->startPose[0] + dx;
            tmpTargetPose[1] = this->startPose[1] + dy;
            tmpTargetPose[2] = this->startPose[2] + dz;

            logger.info("--- startPose ---");
            logger.info(this->startPose[0]);
            logger.info(this->startPose[1]);
            logger.info(this->startPose[2]);
            logger.info("-----");


            tmpTargetPose[3] = a;
            tmpTargetPose[4] = b;
            tmpTargetPose[5] = c;

            logger.info("--- X Y Z A B C ---", false);
            logger.info(tmpTargetPose[0],      false);
            logger.info(tmpTargetPose[1],      false);
            logger.info(tmpTargetPose[2],      false);
            logger.info(tmpTargetPose[3],      false);
            logger.info(tmpTargetPose[4],      false);
            logger.info(tmpTargetPose[5],      false);
            logger.info("-----",               false);

            // v = s/t  t = s/v
            moveInTime = (distance / (float)(totalInterpolationSteps)) / this->maxVelocity;

            unsigned int returnCode = this->_Kinematic.inverse(
                tmpTargetPose[0],
                tmpTargetPose[1],
                tmpTargetPose[2],
                tmpTargetPose[3],
                tmpTargetPose[4],
                tmpTargetPose[5],
                tmpTargetAngles
                );

            if (returnCode == Kinematic::OK) {} else {
                logger.warning("Oh noo, out of bounds");

                // set tartgets to prevent locking of other axis
                this->getCurrentPose(this->targetPose);
                this->getCurrentLogicalAngles(this->targetAngles);
                this->state = IDLE;
                return;
            }

            // todo handle singularity
            if (fabs(tmpTargetAngles[4] - (PI / 2.0)) < 0.05) { // axis 5 and 3 in line
                logger.warning("singularity axis 5,3");
                tmpTargetAngles[3] = this->Servos[3]->getCurrentAngle();
                tmpTargetAngles[5] = this->Servos[5]->getCurrentAngle();
            }

            break;
        }

        case CIRCULAR:
            logger.error("CIRCULAR interpolation not implemented, yet");
            break;
        }

        logger.info("tmpTarget angles 0: " + String(tmpTargetAngles[0] * RAD_TO_DEG));
        logger.info("tmpTarget angles 1: " + String(tmpTargetAngles[1] * RAD_TO_DEG));
        logger.info("tmpTarget angles 2: " + String(tmpTargetAngles[2] * RAD_TO_DEG));
        logger.info("tmpTarget angles 3: " + String(tmpTargetAngles[3] * RAD_TO_DEG));
        logger.info("tmpTarget angles 4: " + String(tmpTargetAngles[4] * RAD_TO_DEG));
        logger.info("tmpTarget angles 5: " + String(tmpTargetAngles[5] * RAD_TO_DEG));


        for (size_t i = 0; i < 6; i++) {
            if ((tmpTargetAngles[i] < this->logicalAngleLimits[i][0]) ||
                (tmpTargetAngles[i] > this->logicalAngleLimits[i][1])) {
                logger.warning("servo " + String(i) + " is out of logic angle: " + String(tmpTargetAngles[i] * RAD_TO_DEG) + " min: "
                               + String(this->logicalAngleLimits[i][0] * RAD_TO_DEG) + " max "
                               + String(this->logicalAngleLimits[i][1] * RAD_TO_DEG) + " " + String(this->logicalAngleLimits[i][1]));

                // set tartgets to prevent locking of other axis
                this->getCurrentPose(this->targetPose);
                this->getCurrentLogicalAngles(this->targetAngles);
                this->state = IDLE;
                return;
            }
        }

        // std::cout << tmpTargetAngles[1] << '\n';
        this->logicalToPhysicalAngles(tmpTargetAngles);

        for (size_t i = 0; i < 6; i++) { // todo remove check here. change notation of logic angles
            // todo pass custom checking function to comprise 4 Axis kinematic coupled robots
            if ((tmpTargetAngles[i] < this->Servos[i]->getMinRadAngle()) || (tmpTargetAngles[i] > this->Servos[i]->getMaxRadAngle())) {
                logger.warning("servo " + String(i) + " is out of physical angle: " + String(tmpTargetAngles[i] / PI * 180) + " min: "
                               + String(this->Servos[i]->getMinRadAngle() / PI * 180) + " max "
                               + String(this->Servos[i]->getMaxRadAngle() / PI * 180));

                // set tartgets to prevent locking of other axis (locking one axis also locks the others unless the locked axis is changed)
                this->getCurrentPose(this->targetPose);
                this->getCurrentLogicalAngles(this->targetAngles);
                this->state = IDLE;
                return;
            }
        }

        float minTime = moveInTime;

        // calculate the minimum time needed based on the servo configuration
        // not working for accelerated movements
        for (unsigned int i = 0; i < 6; i++) {
            float dAngle     = fabs(tmpTargetAngles[i] - this->Servos[i]->getCurrentAngle());
            float moveInTime = dAngle / this->Servos[i]->getMaxAngleVelocity();

            if (moveInTime > minTime) minTime = moveInTime;
        }

        if (minTime > moveInTime) {
            logger.warning("could not move in time: " + String(moveInTime));
            logger.warning(" using instead time: " + String(minTime));
        }

        if (minTime != 0) { // todo why should it be 0?
            for (unsigned int j = 0; j < 6; j++) {
                float dAngle = fabs(tmpTargetAngles[j] - this->Servos[j]->getCurrentAngle());
                this->Servos[j]->setCurrentAngleVelocity(dAngle / minTime);

                // std::cout << "set angle"<< tmpTargetAngles[j]<< '\n';
                this->Servos[j]->setTargetRadAngle(tmpTargetAngles[j]);
            }
        }
        this->state = MOVING;

        break;
    }
    }
}

bool RobotController::_poseEquals(float pos1[6], float pos2[6]) {
    for (unsigned int i = 0; i < 6; i++) {
        if (fabs(pos1[i] - pos2[i]) >= 1e-4) {
            return false;
        }
    }
    return true;
}

bool RobotController::_anglesEqual(float angles1[6], float angles2[6]) {
    for (unsigned int i = 0; i < 6; i++) {
        if (fabs(angles1[i] - angles2[i]) >= 1e-4) {
            return false;
        }
    }
    return true;
}

/**
 * rotate a vector v around an axis unit by angleRad
 * @param ret   returned vector
 * @param unit  the axis to rotate around
 * @param v     the vector to rotate
 * @param angleRad [description]
 */
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

    ADDV(ret1, ret1, ret2);
    ADDV(ret1, ret1, ret3);

    ret[0] = ret1[0];
    ret[1] = ret1[1];
    ret[2] = ret1[2];
}
