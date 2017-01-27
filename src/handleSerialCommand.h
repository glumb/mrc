#include "RingBuffer.h"
#include "IOLogic.h"
#include "Logger.h"
#include "RobotController.h"
#include "AdditionalAxisController.h"

namespace {
Logger logger("handleSerialCommand");
}

#define SRIL_OptionAndValueBufferSize 10

#define SRIL_BUFFER_SIZE 128

#define AUTO_OPEN_FRAME true

#define INPUT_BUFFER_SIZE 80

#define READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE 30

#define DEG_2_RAD ((1f / 180f) * PI)
#define RAD_2_DEG ((1f / PI) * 180)

const unsigned char SRCP_START = 'S';
const unsigned char SRCP_END   = '\r';

const unsigned char SRCP_COMMAND_QUEUE_IN     = 'Q';
const unsigned char SRCP_COMMAND_MOVE_TO      = 'G';
const unsigned char SRCP_COMMAND_HALT         = 'H';
const unsigned char SRCP_COMMAND_GET_ANGLE    = 'L';
const unsigned char SRCP_COMMAND_GET_POSITION = 'P';
const unsigned char SRCP_COMMAND_TEST         = 'T';

const unsigned char SRIL_COMMAND_MOVE     = 'M';
const unsigned char SRIL_COMMAND_LOGIC    = 'L';
const unsigned char SRIL_COMMAND_SET_X    = 'X';
const unsigned char SRIL_COMMAND_SET_Y    = 'Y';
const unsigned char SRIL_COMMAND_SET_Z    = 'Z';
const unsigned char SRIL_COMMAND_SET_A    = 'A';
const unsigned char SRIL_COMMAND_SET_B    = 'B';
const unsigned char SRIL_COMMAND_SET_C    = 'C';
const unsigned char SRIL_COMMAND_ROTATE   = 'R';
const unsigned char SRIL_COMMAND_VELOCITY = 'V';

const unsigned int SRIL_MOVEMENT_METHOD_P2P      = 0;
const unsigned int SRIL_MOVEMENT_METHOD_LINEAR   = 1;
const unsigned int SRIL_MOVEMENT_METHOD_CIRCULAR = 2;

// todo move the parsers to the module Additional axis, Robocon and IOLogic or create a dedicated parser module

class handleSerialCommand {
public:

    handleSerialCommand(RobotController *rc, IOLogic IOLogic, AdditionalAxisController *AxisCon) :
        SRILRingBuffer(SRIL_BUFFER_SIZE),
        robotController(rc), Logic(IOLogic),
        AxisController(AxisCon) {}

    void consumeSRILFromBuffer() {
        unsigned char c      = 0;
        unsigned char symbol = '\0';
        char optionsAndValue[SRIL_OptionAndValueBufferSize];
        unsigned char optionsAndValuePointer = 0;

        // Serial.println("consuming");

        if (SRILRingBuffer.getSize() > 0) { // buffer is not empty
            this->robotController->startTransaction();

            while (c != RingBuffer::END) {
                c = SRILRingBuffer.getByte();
                logger.info("ringBuffer parse char " + String((char)c));


                if (((c >= 65) && (c <= 90)) || (c == RingBuffer::END)) { // is a character or end
                    if (symbol != '\0') {                                 // command finished, interpret it. skip first symbol
                        optionsAndValue[optionsAndValuePointer] = '\0';   // NULL terminate to form a string

                        switch (symbol) {
                        case SRIL_COMMAND_MOVE: {
                            unsigned int movementMethodCode = atoi(optionsAndValue);
                            logger.info("SRIL_COMMAND_MOVE " + String(movementMethodCode));

                            switch (movementMethodCode) {
                            case SRIL_MOVEMENT_METHOD_P2P:
                                logger.info("SRIL_MOVEMENT_METHOD_P2P");

                                robotController->setMovementMethod(RobotController::MOVEMENT_METHODS::P2P);
                                break;

                            case SRIL_MOVEMENT_METHOD_LINEAR:
                                logger.info("SRIL_MOVEMENT_METHOD_LINEAR");
                                robotController->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
                                break;

                            case SRIL_MOVEMENT_METHOD_CIRCULAR:
                                Serial.println("CIRCULAR not implemented");
                                break;
                            }
                            break;
                        }

                        case SRIL_COMMAND_VELOCITY:
                        {
                            float value = atof(optionsAndValue); //  V<val>

                            robotController->setMaxVelocity(value);
                            AxisController->setVelocity(value);
                            break;
                        }

                        case SRIL_COMMAND_SET_X:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting x to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::X, value);
                            break;
                        }

                        case SRIL_COMMAND_SET_Y:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting y to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::Y, value);
                            break;
                        }

                        case SRIL_COMMAND_SET_Z:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting z to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::Z, value);
                            break;
                        }

                        case SRIL_COMMAND_SET_A:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::A, value);
                            break;
                        }

                        case SRIL_COMMAND_SET_B:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::B, value);
                            break;
                        }

                        case SRIL_COMMAND_SET_C:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::C, value);
                            break;
                        }

                        case SRIL_COMMAND_ROTATE:                               // S M R4 1.12345 E
                        {
                            float value         = atof(optionsAndValue + 1);
                            unsigned int option = (int)optionsAndValue[0] - 48; // shift one because option V<opt><val>

                            logger.info("Rotate R" + String(option) + " value " + String(value));

                            // logger.info(String(value));

                            robotController->setTargetAngle(option, value / 180.0 * PI);

                            if (option >= 6) {
                                this->AxisController->setAxisToAngle(option - 6, value / 180.0 * PI);
                            }

                            // servos[option]->setTargetRadAngle(value / 180 * PI);

                            // robotController->setAxisRotation(option,value);
                            break;
                        }

                        case 'U':                                               // U1 00100 #wait for, U0 01020 #output
                        {
                            float value         = atof(optionsAndValue + 1);
                            unsigned int option = (int)optionsAndValue[0] - 48; // shift one because option V<opt><val>

                            if (option == 1) {
                                Serial.println("not implemented");
                            } else {
                                Serial.println(value);
                            }

                            break;
                        }

                        case 'I':                                              // S Q L I1 9 E
                        {                                                      // I<pin><state>
                            unsigned int pin   = (int)optionsAndValue[0] - 48;
                            unsigned int state = (int)optionsAndValue[1] - 48; // shift one because option I<opt><val>

                            Logic.addCondition(pin, state);
                            break;
                        }

                        case 'O':                                              // S Q L O1 8 E     S Q L O0 8 E
                        {                                                      // O<state><pin>
                            unsigned int pin   = (int)optionsAndValue[0] - 48;
                            unsigned int state = (int)optionsAndValue[1] - 48; // shift one because option I<opt><val>

                            Logic.setOutput(pin, state);
                            break;
                        }

                        default:
                            Serial.println("unknown Symbol ");
                            Serial.println((char)symbol);
                        } /* code */
                    } else {
                        // newline?
                    }
                    optionsAndValuePointer = 0;
                    symbol                 = c;
                } else if (((c >= 48) && (c <= 57)) || (c == 46) || (c == 45)) { // is a digit or dot or minus
                    optionsAndValue[optionsAndValuePointer] = c;
                    optionsAndValuePointer++;

                    if (optionsAndValuePointer >= SRIL_OptionAndValueBufferSize) {
                        Serial.println("max option and value size exceeded");
                    }
                } else  {
                    Serial.print("unknown char ");
                    Serial.println((char)c);
                }
            }
            this->robotController->endTransaction();
        }
        logger.info("Exiting consumeSRILFromBuffer");
    }

    void parseCommand(unsigned char buffer[], unsigned int length) {
        if (length == 0) { // dont parse an empty command
            return;
        }

        switch (buffer[0]) {
        case SRCP_COMMAND_MOVE_TO:
        {
            unsigned int status;                                             // todo put in class
            status = SRILRingBuffer.putBytesInFront(buffer + 1, length - 1); // queue in SRIL minus SRCP command

            if (status == RingBuffer::STATUS_FULL) {
                logger.warning("buffer full");
            } else {
                logger.info("command in buffer");
            }

            break;
        }

        case SRCP_COMMAND_HALT:
        case SRCP_COMMAND_GET_ANGLE:

        case SRCP_COMMAND_GET_POSITION:
        {
            float currentPose[6];
            robotController->getCurrentPose(currentPose);
            Serial.print("x ");
            Serial.println(currentPose[0]);
            Serial.print("y ");
            Serial.println(currentPose[1]);
            Serial.print("z ");
            Serial.println(currentPose[2]);
            Serial.print("a ");
            Serial.println(currentPose[3]);
            Serial.print("b ");
            Serial.println(currentPose[4]);
            Serial.print("c ");
            Serial.println(currentPose[5]);
            break;
        }

        case SRCP_COMMAND_TEST:

            for (size_t i = 1; i < length; i++) {
                Serial.print((char)buffer[i]);
            }

            break;

        case SRCP_COMMAND_QUEUE_IN:
        {
            unsigned int status;                                      // todo put in class
            status = SRILRingBuffer.putBytes(buffer + 1, length - 1); // queu in SRIL minus SRCP command

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
                status = SRILRingBuffer.putBytes(buffer, length); // queu in SRIL minus SRCP command
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

    char toUpper(char ch1)
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

    void parseSRCP(byte incomingByte) {
        static bool frameStarted = false;
        static unsigned char inputByteBuffer[INPUT_BUFFER_SIZE];
        static unsigned int  inputByteBufferPointer = 0;

        // read the incoming byte:
        incomingByte = toUpper(incomingByte);

        if ((AUTO_OPEN_FRAME && !frameStarted) || (incomingByte == SRCP_START)) {
            logger.info("Frame start");
            frameStarted           = true;
            inputByteBufferPointer = 0;
        }

        if (frameStarted) {
            if (incomingByte == SRCP_START) {
                // dont save the start byte
            } else if (incomingByte == SRCP_END) { // message complete. write to messagequeue
                // logger.time("beforeee parseCommand");
                logger.info("Frame end");
                inputByteBuffer[inputByteBufferPointer + 1] = 0;
                logger.info(inputByteBuffer);

                // logger.time("before parseCommand");
                parseCommand(inputByteBuffer, inputByteBufferPointer);
                frameStarted = false;
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
                    logger.warning("input buffer full! pointer: " + String(
                                       inputByteBufferPointer) + " size: " + INPUT_BUFFER_SIZE + " (frame too long)");
                }
            } else {
                //  logger.info("I received unknow char: " +String((char)incomingByte)+" ["+String(incomingByte)+"]");
            }
        }
    }

    void listen() {
        unsigned int bytesRead = 0;

        logger.resetTime();

        while (Serial.available() && bytesRead < READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE) {
            // logger.time("before serial read");
            parseSRCP(Serial.read());

            // logger.time("after serial read");
            bytesRead++;
        }


        if (SRCPMODE != HALT) {
            if (Logic.isDone() && !robotController->isMoving() && (SRILRingBuffer.getSize() > 0)) { // todo add additionalAxis
                logger.info("Consuming from buffer");
                consumeSRILFromBuffer();
                Serial.println(String("$$") + SRILRingBuffer.getSize());
            }
        }
    }

    int getBufferSize() {
        return SRIL_BUFFER_SIZE;
    }

    int getFullBufferSize() {
        return SRILRingBuffer.getSize();
    }

private:

    enum srcpMode { HALT, GO };
    srcpMode SRCPMODE = GO;

    RobotController *robotController;
    AdditionalAxisController *AxisController;
    IOLogic Logic;
    RingBuffer SRILRingBuffer; // todo does not report if full
    // Simple/Sophisticated Robot Instruction Language
};
