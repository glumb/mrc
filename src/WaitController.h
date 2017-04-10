#include "Arduino.h"
#ifndef WAIT_CONTROLLER_H
#define WAIT_CONTROLLER_H 1


class WaitController {
public:

    WaitController() {}

    void waitMs(unsigned long ms) {
        this->waitUntil = millis() + ms;
    }

    bool isDone() {
        return millis() > this->waitUntil;
    }

private:

    unsigned long waitUntil;
};

#endif // ifndef WAIT_CONTROLLER_H
