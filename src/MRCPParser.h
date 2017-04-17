#ifndef MRCPPARSER_H
#define MRCPPARSER_H 1

#include "RingBuffer.h"
#include "EEPromStorage.h"
#include "MRILParser.h"
#include "CommunicationInterface.h"

#define MRIL_OptionAndValueBufferSize 10

#define MRIL_BUFFER_SIZE 228

#define AUTO_OPEN_FRAME true

#define INPUT_BUFFER_SIZE 80

#define READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE 30

class MRCPParser {
public:

    MRCPParser(EEPromStorage         & _EEPromStorage,
               RingBuffer            & _RingBuffer,
               MRILParser            & _MRILParser,
               CommunicationInterface& _CommunicationInterface);

    void parseCommand(char         buffer[],
                      unsigned int length);

    void parseMRCP(char incomingByte);

    void listen();
    int  getBufferSize();

    int  getFullBufferSize();

    void sendMessage(String message);

private:

    EEPromStorage& _EEPromStorage;
    RingBuffer& _RingBuffer;
    MRILParser& _MRILParser;
    CommunicationInterface& _CommunicationInterface;

    enum mrcpMode { HALT, QUEUE, EEPROM, EXECUTE };
    mrcpMode MRCPMODE = EXECUTE;

    long commandNumber = 0;


    char toUpper(char ch1);
};

#endif // ifndef MRCPPARSER_H
