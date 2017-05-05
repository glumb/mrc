#ifndef MRCPR_H
#define MRCPR_H 1

#ifndef MOCK_VIRTUAL // used for setting methods to virtual in test environment
#define MOCK_VIRTUAL
#endif

// #include "SerialIO.h"
#include "Logger.h"
#include "MRCP.h"
#include "Arduino.h"
#include "CommunicationInterface.h"


// todo move the parsers to the module Additional axis, Robocon and IOLogic or create a dedicated parser module

class MRCPR {
public:

    MRCPR(CommunicationInterface& _IO):_IO(_IO) {}


    MOCK_VIRTUAL void sendMessage(String message) {
        this->_IO.transmit(MRCP_START_FRAME);
        this->_IO.transmit(message);
        this->_IO.transmit(MRCP_END_FRAME);
    }

private:
  CommunicationInterface& _IO;
};

#endif // ifndef MRCPR_H
