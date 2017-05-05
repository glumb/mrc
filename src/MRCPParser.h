#ifndef MRCPPARSER_H
#define MRCPPARSER_H 1

#include "RingBuffer.h"
#include "EEPromStorage.h"
#include "MRILParser.h"
#include "MRCPR.h"

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
               MRCPR& _MRCPR);

    void parseCommand(char         buffer[],
                      unsigned int length);

    void parseChar(char incomingByte);

    void process();
    int  getBufferSize();

    int  getFullBufferSize();

    void sendMessage(String message);

private:

    EEPromStorage& _EEPromStorage;
    RingBuffer& _RingBuffer;
    MRILParser& _MRILParser;
    MRCPR& _MRCPR;

    enum MRCPMODE { HALT, QUEUE, EEPROM, EXECUTE };
    MRCPMODE mrcpMode = EXECUTE;

    long commandNumber = 0;


    char toUpper(char ch1);
};

#endif // ifndef MRCPPARSER_H
