#ifndef MRCPR_H
#define MRCPR_H 1

// #include "SerialIO.h"
#include "Logger.h"
#include "MRCP.h"
#include "Arduino.h"
#include "SerialIO.h"






// todo move the parsers to the module Additional axis, Robocon and IOLogic or create a dedicated parser module

class MRCPR {
public:

    MRCPR() {}


    void sendMessage(String message) {
        IO.transmit(MRCP_START_FRAME);
        IO.transmit(message);
        IO.transmit(MRCP_END_FRAME);
    }

private:

};

#endif
