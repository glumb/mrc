#include "RingBuffer.h"
#include "IOLogic.h"
#include "Logger.h"
#include "RobotController.h"
#include "AdditionalAxisController.h"
#include "WaitController.h"

namespace {
Logger logger("handleSerialCommand");
}

#define MRIL_OptionAndValueBufferSize 10

#define MRIL_BUFFER_SIZE 228

#define AUTO_OPEN_FRAME true

#define INPUT_BUFFER_SIZE 80

#define READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE 30

#define DEG_2_RAD ((1f / 180f) * PI)
#define RAD_2_DEG ((1f / PI) * 180)


// const  char MRCP_END_FRAME   = '\r';
const  char MRCP_START_FRAME = 'S';
const  char MRCP_END_FRAME   = 'E';

const  char MRCP_COMMAND_QUEUE_IN = 'Q';
const  char MRCP_COMMAND_EXECUTE  = 'E';
const  char MRCP_COMMAND_WRITE    = 'W';


const  char MRIL_COMMAND_SET_X           = 'X';
const  char MRIL_COMMAND_SET_Y           = 'Y';
const  char MRIL_COMMAND_SET_Z           = 'Z';
const  char MRIL_COMMAND_SET_A           = 'A';
const  char MRIL_COMMAND_SET_B           = 'B';
const  char MRIL_COMMAND_SET_C           = 'C';
const  char MRIL_COMMAND_ROTATE          = 'R';
const  char MRIL_COMMAND_VELOCITY        = 'V';
const  char MRIL_COMMAND_ANCHOR          = 'T';
const  char MRIL_COMMAND_MOVEMENT_METHOD = 'M';
const  char MRIL_COMMAND_LOGIC_INPUT     = 'I';
const  char MRIL_COMMAND_LOGIC_OUTPUT    = 'O';
const  char MRIL_COMMAND_SERIAL_IO       = 'U';

const  char MRIL_COMMAND_WAIT = 'W';

const  char MRIL_COMMAND_TEST = 'F';

const  char MRIL_COMMAND_NUMBER = 'N';

const  char MRIL_COMMAND_HALT = 'H';
const  char MRIL_IS_MOVING    = 'K';

const unsigned int MRIL_MOVEMENT_METHOD_P2P      = 0;
const unsigned int MRIL_MOVEMENT_METHOD_LINEAR   = 1;
const unsigned int MRIL_MOVEMENT_METHOD_CIRCULAR = 2;



// todo move the parsers to the module Additional axis, Robocon and IOLogic or create a dedicated parser module

class handleSerialCommand {
public:

    handleSerialCommand(RobotController *rc, IOLogic IOLogic, AdditionalAxisController *AxisCon, WaitController WaitCon) :
        MRILRingBuffer(MRIL_BUFFER_SIZE),
        robotController(rc),
        Logic(IOLogic),
        AxisController(AxisCon),
        WaitCon(WaitCon) {}

    void consumeMRILFromBuffer() {
        this->commandNumber = -1;

        unsigned char c      = 0;
        unsigned char symbol = '\0';
        char optionsAndValue[MRIL_OptionAndValueBufferSize];
        unsigned char optionsAndValuePointer = 0;

        // Serial.println("consuming");

        if (MRILRingBuffer.getSize() > 0) { // buffer is not empty
            this->robotController->startTransaction();

            while (c != RingBuffer::END) {
                c = MRILRingBuffer.getByte();
                logger.info("ringBuffer parse char " + String((char)c));


                if (((c >= 65) && (c <= 90)) || (c == RingBuffer::END)) { // is a character or end
                    if (symbol != '\0') {                                 // command finished, interpret it. skip first symbol
                        optionsAndValue[optionsAndValuePointer] = '\0';   // NULL terminate to form a string

                        switch (symbol) {
                        case MRIL_COMMAND_MOVEMENT_METHOD: {
                            unsigned int movementMethodCode = atoi(optionsAndValue);
                            logger.info("MRIL_COMMAND_MOVE " + String(movementMethodCode));

                            switch (movementMethodCode) {
                            case MRIL_MOVEMENT_METHOD_P2P:
                                logger.info("MRIL_MOVEMENT_METHOD_P2P");

                                robotController->setMovementMethod(RobotController::MOVEMENT_METHODS::P2P);
                                break;

                            case MRIL_MOVEMENT_METHOD_LINEAR:
                                logger.info("MRIL_MOVEMENT_METHOD_LINEAR");
                                robotController->setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
                                break;

                            case MRIL_MOVEMENT_METHOD_CIRCULAR:
                                Serial.println("CIRCULAR not implemented");
                                break;
                            }
                            break;
                        }

                        case MRIL_COMMAND_VELOCITY:
                        {
                            float value = atof(optionsAndValue); //  V<val>

                            robotController->setMaxVelocity(value);
                            AxisController->setVelocity(value);
                            break;
                        }

                        case MRIL_COMMAND_SET_X:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting x to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::X, value);
                            break;
                        }

                        case MRIL_COMMAND_SET_Y:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting y to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::Y, value);
                            break;
                        }

                        case MRIL_COMMAND_SET_Z:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            logger.info("setting z to: " + String(value));

                            robotController->setTargetPose(RobotController::POSITION::Z, value);
                            break;
                        }

                        case MRIL_COMMAND_SET_A:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::A, value);
                            break;
                        }

                        case MRIL_COMMAND_SET_B:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::B, value);
                            break;
                        }

                        case MRIL_COMMAND_SET_C:
                        {
                            float value = atof(optionsAndValue); // shift one because option V<opt><val>
                            Serial.println(value);

                            robotController->setTargetPose(RobotController::POSITION::C, value);
                            break;
                        }

                        case MRIL_COMMAND_ROTATE:                               // S M R4 1.12345 E
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

                        case MRIL_COMMAND_LOGIC_INPUT:                         // S Q L I1 9 E
                        {                                                      // I<pin><state>
                            unsigned int pin   = (int)optionsAndValue[0] - 48;
                            unsigned int state = (int)optionsAndValue[1] - 48; // shift one because option I<opt><val>

                            Logic.addCondition(pin, state);
                            break;
                        }

                        case MRIL_COMMAND_LOGIC_OUTPUT:                        // S Q L O1 8 E     S Q L O0 8 E
                        {                                                      // O<state><pin>
                            unsigned int pin   = (int)optionsAndValue[0] - 48;
                            unsigned int state = (int)optionsAndValue[1] - 48; // shift one because option I<opt><val>

                            Logic.setOutput(pin, state);
                            break;
                        }

                        case MRIL_COMMAND_WAIT:
                        { // W<ms>
                            unsigned long ms = atol(optionsAndValue);

                            this->WaitCon.waitMs(ms);
                            break;
                        }

                        case MRIL_COMMAND_NUMBER:
                        { // N<number>
                          // float value = atof(optionsAndValue);
                            this->commandNumber = atoi(optionsAndValue);
                            this->sendMessage(String(MRIL_COMMAND_NUMBER) + "0" + String(this->commandNumber));

                            break;
                        }

                        case MRIL_COMMAND_TEST:

                            break;

                        case MRIL_COMMAND_HALT:
                            break;

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

                    if (optionsAndValuePointer >= MRIL_OptionAndValueBufferSize) {
                        Serial.println("max option and value size exceeded");
                    }
                } else  {
                    Serial.print("unknown char ");
                    Serial.println((char)c);
                }
            }
            this->robotController->endTransaction();
        }
        logger.info("Exiting consumeMRILFromBuffer");
    }

    void parseCommand(unsigned char buffer[], unsigned int length) {
        if (length == 0) { // dont parse an empty command
            return;
        }

        switch (buffer[0]) {
        case MRCP_COMMAND_EXECUTE:
        {
            unsigned int status;                                             // todo put in class
            status = MRILRingBuffer.putBytesInFront(buffer + 1, length - 1); // queue in MRIL minus MRCP command

            if (status == RingBuffer::STATUS_FULL) {
                logger.warning("buffer full");
            } else {
                logger.info("command in buffer");
            }

            break;
        }


        case MRCP_COMMAND_QUEUE_IN:
        {
            unsigned int status;                                      // todo put in class
            status = MRILRingBuffer.putBytes(buffer + 1, length - 1); // queu in MRIL minus MRCP command

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
                status = MRILRingBuffer.putBytes(buffer, length); // queu in MRIL minus MRCP command
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

    void parseMRCP(byte incomingByte) {
        static bool frameStarted = false;
        static unsigned char inputByteBuffer[INPUT_BUFFER_SIZE];
        static unsigned int  inputByteBufferPointer = 0;
        static bool inputBufferFull                 = false;

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

    void listen() {
        unsigned int bytesRead = 0;

        logger.resetTime();

        while (Serial.available() && bytesRead < READ_NUMBER_OF_BYTES_FROM_SERIAL_BEFORE_CONTINUE) {
            // logger.time("before serial read");
            parseMRCP(Serial.read());

            // logger.time("after serial read");
            bytesRead++;
        }


        if (MRCPMODE != HALT) {
            if (this->Logic.isDone() && !this->robotController->isMoving() &&  this->WaitCon.isDone()) { // todo add additionalAxis
                if (this->commandNumber > 0) {                                                           // command number exists
                    this->sendMessage(String(MRIL_COMMAND_NUMBER) + "1" + String(this->commandNumber));
                    this->commandNumber = -1;                                                            // command was executed
                }

                if (this->MRILRingBuffer.getSize() > 0) {
                    logger.info("Consuming from buffer");
                    consumeMRILFromBuffer();
                    Serial.println(String("$$") + MRILRingBuffer.getSize());
                }
            }
        }
    }

    int getBufferSize() {
        return MRIL_BUFFER_SIZE;
    }

    int getFullBufferSize() {
        return MRILRingBuffer.getSize();
    }

private:

    enum srcpMode { HALT, GO };
    srcpMode MRCPMODE = GO;

    long commandNumber = 0;

    RobotController *robotController;
    AdditionalAxisController *AxisController;
    IOLogic Logic;
    WaitController WaitCon;
    RingBuffer MRILRingBuffer; // todo does not report if full
    // Simple/Sophisticated Robot Instruction Language

    void sendMessage(String message) {
        Serial.print(MRCP_START_FRAME);
        Serial.print(message);
        Serial.print(MRCP_END_FRAME);
    }
};
