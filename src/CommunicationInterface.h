#ifndef COMMUNICATIONINTERFACE_H
#define COMMUNICATIONINTERFACE_H 1

#include "Arduino.h"

class CommunicationInterface {
public:

    virtual ~CommunicationInterface() {}

    virtual void process()                            {}

    virtual void onData(void (*dataCallback)(char))   {}


    virtual void transmit(char message[], int length) {}

    virtual void transmit(char string[])              {}

    virtual void transmit(char c)                     {}

    virtual void transmit(String string)              {}
};
#endif // ifndef COMMUNICATIONINTERFACE_H
