#include "MRCPParser.h"
#include "RingBuffer.h"
#include "Logger.h"
#include "EEPromStorage.h"
#include "MRILParser.h"

namespace {
static Logger logger("MRCPParser");
}


MRCPParser::MRCPParser(EEPromStorage& _EEPromStorage,
                       RingBuffer   & _RingBuffer,
                       MRILParser   & _MRILParser,
                       MRCPR        & _MRCPR) :
    _EEPromStorage(_EEPromStorage),
    _RingBuffer(_RingBuffer),
    _MRILParser(_MRILParser),
    _MRCPR(_MRCPR) {
    if (this->_EEPromStorage.getNumberOfMessages() > 0) {
        this->mrcpMode = MRCPMODE::EEPROM;
    }
}

void MRCPParser::parseCommand(char buffer[], unsigned int length) {
    if (length == 0) { // dont parse an empty command
        return;
    }

    switch (buffer[0]) {
    case MRCP_GET_BUFFER_SIZE:
    {
        this->sendFreeBufferSize();

        break;
    }

    case MRCP_COMMAND_EXECUTE:
    {
        // this->mrcpMode = MRCPMODE::EXECUTE; // Executing a read command should not change the current mode
        // could add a special "unobtrusive" mode that does not change the mode either
        _MRILParser.parse(buffer + 1, length - 1);

        break;
    }

    case MRCP_COMMAND_QUEUE_IN:
    {
        this->mrcpMode = MRCPMODE::QUEUE;

        if (length == 1) {
            _RingBuffer.clear();
            logger.info("clearing ringbuffer");
        } else {
            unsigned int status;
            status = _RingBuffer.putBytes(buffer + 1, length - 1); // queue in MRIL minus MRCP command

            if (status == RingBuffer::STATUS_FULL) {
                logger.warning("buffer full");
            } else {
                logger.info("command in buffer");
            }
        }

        break;
    }

    case MRCP_COMMAND_WRITE:
    {
        if (length == 1) {
            _EEPromStorage.clear();
            logger.info("clearing eeprom");
        } else {
            logger.info("writing to eeprom");
            this->mrcpMode = MRCPMODE::EEPROM;
            _EEPromStorage.appendMessage(buffer + 1, length - 1); // queue in MRIL minus MRCP command
        }
        break;
    }

    default:
        logger.info("using default command (queue in): ");
        logger.info((char)buffer[0]);
        {
            this->mrcpMode = MRCPMODE::QUEUE;
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

void MRCPParser::parseChar(char incomingByte) {
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
        static bool isComment = false;

        if (incomingByte == MRCP_END_FRAME) { // message complete. write to messagequeue
            // logger.time("beforeee parseCommand");
            logger.info("Frame end");
            inputByteBuffer[inputByteBufferPointer + 1] = 0;
            logger.info(inputByteBuffer);

            // logger.time("before parseCommand");
            // discard corrupted frame
            if (!inputBufferFull) parseCommand(inputByteBuffer, inputByteBufferPointer);
            frameStarted    = false;
            inputBufferFull = false;
            isComment       = false;
        } else if ((incomingByte == '#') || (incomingByte == '(') || isComment) { // dont parse symbols after comment. Order matters.
                                                                                  // EndByte has to be parsed
            logger.info("isComment: " + String(incomingByte));
            isComment = true;
        } else if (incomingByte == MRCP_START_FRAME) {
            // dont save the start byte
        } else if (((incomingByte >= 48) && (incomingByte <= 57)) || // numbers
                   ((incomingByte >= 65) && (incomingByte <= 90)) || // letters
                   ((incomingByte >= 43) && (incomingByte <= 46)))   // + . , -
        {
            inputByteBuffer[inputByteBufferPointer] = incomingByte;
            inputByteBufferPointer++;

            // say what you got:
            // logger.info("I received: " +String((char)incomingByte)+" ["+String(incomingByte)+"]");

            if (inputByteBufferPointer >= INPUT_BUFFER_SIZE) { // ge because we need a null for last char
                // frame too long
                inputBufferFull = true;
                logger.warning("input buffer full! pointer: " + String(
                                   inputByteBufferPointer) + " size: " + String(INPUT_BUFFER_SIZE) + " (frame too long)");
            }
        } else {
            logger.info("I received unknow char: " + String((char)incomingByte) + " [" + String(incomingByte) + "]");
        }
    }
}

void MRCPParser::process() {
    if (this->mrcpMode != MRCPMODE::HALT) {
        if (this->_MRILParser.isDone()) { // todo add additionalAxis
            switch (this->mrcpMode) {
            case QUEUE:

                if (this->_RingBuffer.getSize() > 0) {
                    logger.info("Consuming from buffer");

                    char mrilInstruction[INPUT_BUFFER_SIZE];

                    this->sendFreeBufferSize();
                    char length = _RingBuffer.getMessage(mrilInstruction);

                    this->_MRILParser.parse(mrilInstruction, length);
                }
                break;

            case EEPROM:
                char mrilInstruction[INPUT_BUFFER_SIZE];
                char length = this->_EEPromStorage.getNextMessage(mrilInstruction);

                if (length != 0) {
                    this->_MRILParser.parse(mrilInstruction, length);
                }

                break;
            }
        }
    }
}

MRCPParser::MRCPMODE MRCPParser::getMode() {
    return this->mrcpMode;
}

void MRCPParser::sendFreeBufferSize() {
    this->_MRCPR.sendMessage(String(MRCP_GET_BUFFER_SIZE) + String(this->_RingBuffer.getCapacity() - this->_RingBuffer.getSize()));
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
