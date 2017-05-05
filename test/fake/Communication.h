#ifndef FAKE_COMMUNICATION_H
#define FAKE_COMMUNICATION_H 1

#include "Arduino.h"
#include "CommunicationInterface.h"

class Communication: public CommunicationInterface {
public:

    Communication() {};
    ~Communication() {};

    void process()                            {};

    void onData(void (*dataCallback)(char))   {};


    void transmit(char message[], int length) {};

    void transmit(const char string[])              {};

    void transmit(char c)                     {};

    void transmit(String string)              {};
};
#endif // ifndef COMMUNICATIONINTERFACE_H
