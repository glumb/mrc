#ifndef MRCPPARSER_H
#define MRCPPARSER_H 1

#include "RingBuffer.h"
#include "EEPromStorage.h"
#include "MRILParser.h"
#include "MRCPR.h"

#define MRIL_OptionAndValueBufferSize 10

#define MRIL_BUFFER_SIZE 228

#define AUTO_OPEN_FRAME true

#define INPUT_BUFFER_SIZE 80 // maximum number of bytes in a MRCP instruction

#define READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE 30


#ifndef MOCK_VIRTUAL // used for setting methods to virtual in test environment
# define MOCK_VIRTUAL
#endif // ifndef MOCK_VIRTUAL

class MRCPParser {
public:

    MRCPParser(EEPromStorage& _EEPromStorage,
               RingBuffer   & _RingBuffer,
               MRILParser   & _MRILParser,
               MRCPR        & _MRCPR);

    enum MRCPMODE { HALT, QUEUE, EEPROM, EXECUTE };

    MOCK_VIRTUAL void parseCommand(char         buffer[],
                                   unsigned int length);

    MOCK_VIRTUAL void parseChar(char incomingByte);

    void              process();


    void              sendMessage(String message);

    MRCPMODE          getMode();

private:

    EEPromStorage& _EEPromStorage;
    RingBuffer& _RingBuffer;
    MRILParser& _MRILParser;
    MRCPR& _MRCPR;

    MRCPMODE mrcpMode = EXECUTE;

    long commandNumber = 0;


    char toUpper(char ch1);

    void sendFreeBufferSize();
};

#endif // ifndef MRCPPARSER_H
