#ifndef MOCK_MRCPR_H
#define MOCK_MRCPR_H 1


#include "MRCPR.h"
#include "Arduino.h"
#include "CommunicationInterface.h"
#include "gmock/gmock.h"


class mock_MRCPR : public MRCPR {
public:

    mock_MRCPR(CommunicationInterface& _IO) : MRCPR(_IO){}

    MOCK_METHOD1(sendMessage, void(String msg));
};

#endif // ifndef MRCPR_H
