#ifndef ADDITIONAL_AXIS_CONTROLLER_H
#define ADDITIONAL_AXIS_CONTROLLER_H 1

class AdditionalAxisController {
public:

    // todo convert velocity from deg to rad
    explicit AdditionalAxisController(VarSpeedServo *Servos[2]) {
        for (size_t i = 0; i < 2; i++) {
            this->servos[i] = Servos[i];
        }
    }

    void setAxisToAngle(unsigned int index, float angle) {
        this->servos[index]->setTargetRadAngle(angle);
    }

    float getCurrentAngle(unsigned int index) {
        return this->servos[index]->getCurrentAngle();
    }

    void setVelocity(float velocity) {
        for (size_t i = 0; i < 2; i++) {
            this->servos[i]->setCurrentAngleVelocity(velocity);
        }
    }

private:

    VarSpeedServo *servos[2];
};

#endif // ifndef ADDITIONAL_AXIS_CONTROLLER_H
