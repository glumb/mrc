#include "MRCPParser.h"
#include "RingBuffer.h"
#include "Logger.h"
#include "EEPromStorage.h"
#include "MRILParser.h"

namespace {
static Logger logger("MRCPParser");
}


MRCPParser::MRCPParser(EEPromStorage _EEPromStorage, RingBuffer _RingBuffer, MRILParser _MRILParser,CommunicationInterface _CommunicationInterface) :
    _EEPromStorage(_EEPromStorage),
    _RingBuffer(_RingBuffer),
    _MRILParser(_MRILParser),
            _CommunicationInterface(_CommunicationInterface) {}

void MRCPParser::parseCommand(char buffer[], unsigned int length) {
    if (length == 0) { // dont parse an empty command
        return;
    }

    switch (buffer[0]) {
    case MRCP_COMMAND_EXECUTE:
    {
        unsigned int status;                                          // todo put in class
        status = _RingBuffer.putBytesInFront(buffer + 1, length - 1); // queue in MRIL minus MRCP command

        if (status == RingBuffer::STATUS_FULL) {
            logger.warning("buffer full");
        } else {
            logger.info("command in buffer");
        }

        break;
    }


    case MRCP_COMMAND_QUEUE_IN:
    {
        unsigned int status;                                   // todo put in class
        status = _RingBuffer.putBytes(buffer + 1, length - 1); // queu in MRIL minus MRCP command

        if (status == RingBuffer::STATUS_FULL) {
            logger.warning("buffer full");
        } else {
            logger.info("command in buffer");
        }

        break;
    }

    case MRCP_COMMAND_WRITE:
    {
        unsigned int status;                                   // todo put in class
        status = _RingBuffer.putBytes(buffer + 1, length - 1); // queu in MRIL minus MRCP command

        if (status == RingBuffer::STATUS_FULL) {
            logger.warning("buffer full");
        } else {
            logger.info("command in buffer");
        }

        break;
    }

    default:
        logger.info("using default command (queue in): ");
        logger.info((char)buffer[0]);
        {
            unsigned int status;

            // logger.time("before ringbuffer");                         // todo put in class
            status = _RingBuffer.putBytes(buffer, length); // queu in MRIL minus MRCP command
            // logger.time("after ringbuffer");                         // todo put in class

            if (status == RingBuffer::STATUS_FULL) {
                logger.warning("buffer full");
            } else {
                logger.info("command in buffer");
            }

            break;
        }
    }
}

void MRCPParser::parseMRCP(char incomingByte) {
    static bool frameStarted = false;
    static char inputByteBuffer[INPUT_BUFFER_SIZE];
    static unsigned int inputByteBufferPointer = 0;
    static bool inputBufferFull                = false;

    // read the incoming byte:
    incomingByte = toUpper(incomingByte);

    if ((AUTO_OPEN_FRAME && !frameStarted) || (incomingByte == MRCP_START_FRAME)) {
        logger.info("Frame start");
        frameStarted           = true;
        inputByteBufferPointer = 0;
    }

    if (frameStarted) {
        if (incomingByte == MRCP_START_FRAME) {
            // dont save the start byte
        } else if (incomingByte == MRCP_END_FRAME) { // message complete. write to messagequeue
            // logger.time("beforeee parseCommand");
            logger.info("Frame end");
            inputByteBuffer[inputByteBufferPointer + 1] = 0;
            logger.info(inputByteBuffer);

            // logger.time("before parseCommand");
            // discard corrupted frame
            if (!inputBufferFull) parseCommand(inputByteBuffer, inputByteBufferPointer);
            frameStarted    = false;
            inputBufferFull = false;
        } else if (((incomingByte >= 48) && (incomingByte <= 57)) || // numbers
                   ((incomingByte >= 65) && (incomingByte <= 90)) || // letters
                   ((incomingByte >= 43) && (incomingByte <= 46)))   // + . , -
        {
            inputByteBuffer[inputByteBufferPointer] = incomingByte;
            inputByteBufferPointer++;

            // say what you got:
            // logger.info("I received: " +String((char)incomingByte)+" ["+String(incomingByte)+"]");
            // logger.info();
            // logger.info(String(incomingByte));

            if (inputByteBufferPointer >= INPUT_BUFFER_SIZE) { // ge because we need a null for last char
                // frame too long
                inputBufferFull = true;
                logger.warning("input buffer full! pointer: " + String(
                                   inputByteBufferPointer) + " size: " + INPUT_BUFFER_SIZE + " (frame too long)");
            }
        } else {
            //  logger.info("I received unknow char: " +String((char)incomingByte)+" ["+String(incomingByte)+"]");
        }
    }
}

void MRCPParser::listen() {
    if (MRCPMODE != HALT) {
        if (this->_MRILParser.isDone()) { // todo add additionalAxis
        }
    }

    switch (MRCPMODE) {
    case QUEUE:

        if (this->_RingBuffer.getSize() > 0) {
            logger.info("Consuming from buffer");

            if (_RingBuffer.getSize() > 0) { // buffer is not empty
                char mrilInstruction[INPUT_BUFFER_SIZE];

                char length = _RingBuffer.getMessage(mrilInstruction);

                this->_MRILParser.parse(mrilInstruction, length);

                // Serial.println(String("$$") + _RingBuffer.getSize());
            }
        }
    }
}

int MRCPParser::getBufferSize() {
    return MRIL_BUFFER_SIZE;
}

int MRCPParser::getFullBufferSize() {
    return _RingBuffer.getSize();
}

void MRCPParser::sendMessage(String message) {
    Serial.print(MRCP_START_FRAME);
    Serial.print(message);
    Serial.print(MRCP_END_FRAME);
}

char MRCPParser::toUpper(char ch1)
{
    char ch2;

    if ((ch1 >= 'a') && (ch1 <= 'z')) {
        ch2 = ('A' + ch1 - 'a');
        return ch2;
    }
    else {
        ch2 = ch1;
        return ch2;
    }
}
