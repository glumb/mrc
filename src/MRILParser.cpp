#include "MRILParser.h"
#include "IOLogic.h"
#include "Logger.h"
#include "RobotController.h"
#include "AdditionalAxisController.h"
#include "WaitController.h"
#include "MRCPR.h"

namespace {
Logger _logger("MRILParser");
}

MRILParser::MRILParser(RobotController         & _RobotController,
                       IOLogic                 & _IOLogic,
                       AdditionalAxisController& _AdditionalAxisController,
                       WaitController          & _WaitController,
                       MRCPR                   & _MRCPR) :
    _RobotController(_RobotController),
    _IOLogic(_IOLogic),
    _AdditionalAxisController(_AdditionalAxisController),
    _WaitController(_WaitController),
    _MRCPR(_MRCPR) {}

void MRILParser::parse(char mrilInstruction[], unsigned int length) {
    // cN changes when a non read Instruction is executed.
    int tmpCommandNumber = -1;

    enum CommandTypes { READ, WRITE, NONE };
    CommandTypes commandType     = CommandTypes::NONE; // 1:write, -1:read  you must not mix read and write commands
    CommandTypes lastCommandType = CommandTypes::NONE;

    char command[MRIL_COMMAND_SIZE];
    unsigned int commandPointer = 0;
    String responseBuffer = "";


    if (length == 0) { // buffer is not empty
        return;
    }

    // todo use atomic instead to also not invoke timers when setting axis R0 R1 and so on....
    this->_RobotController.startTransaction();

    for (size_t i = 0; i < length; i++) { // l:6 i:0    c:90
        char c = mrilInstruction[i];      // i:1 c:51  - i:2 c:65
        // char nextChar        = (i == length - 1) ? 3 : mrilInstruction[i + 1]; // ETX
        bool commandFinished = (i == length - 1) ? true :
                               ((mrilInstruction[i + 1] >= 65) && (mrilInstruction[i + 1] <= 90)) ? true :
                               (mrilInstruction[i + 1] == '#' || mrilInstruction[i + 1] == '(');

        if ((c == '#') || (c == '(')) { // starting comment
            Serial.println("comment-breaking");
            break;                      // do not interpret any other symbols for this instruction
        }


        // is a digit or dot or minus or uppercase letter
        if (((c >= 48) && (c <= 57)) || (c == 46) || (c == 45) || ((c >= 65) && (c <= 90))) {
            command[commandPointer] = c;
            commandPointer++;

            if (commandPointer >= MRIL_COMMAND_SIZE) {
                _logger.warning("max option and value size exceeded");
            }
        } else  {
            // do not parse unknown chars
            _logger.warning("unknown char ");
            _logger.warning((char)c);
            continue;
        }

        _logger.info("ringBuffer parse char " + String((char)c));


        // command finished, interpret it.
        if (commandFinished) {                                                              // is a character or end
            // no options means read command
            commandType = (commandPointer == 1) ? CommandTypes::READ : CommandTypes::WRITE; // todo make decision for individual symbol

            if (command[0] == MRIL_COMMAND_NUMBER) {
                commandType = CommandTypes::NONE;
            }

            if ((commandType != lastCommandType) && (lastCommandType != CommandTypes::NONE)) {
                _logger.warning("Do not mix read and write instructions!");
            } else {
                lastCommandType = commandType;
            }

            char symbol = command[0];

            command[commandPointer] = '\0'; // NULL terminate to form a string

            switch (symbol) {
            case MRIL_COMMAND_MOVEMENT_METHOD: {
                if (commandType == CommandTypes::READ) {
                    responseBuffer += (String(MRIL_COMMAND_MOVEMENT_METHOD) + String(this->_RobotController.getMovementMethod()));
                } else {
                    unsigned int movementMethodCode = atoi(command + 1);
                    _logger.info("MRIL_COMMAND_MOVE " + String(movementMethodCode));

                    switch (movementMethodCode) {
                    case MRIL_MOVEMENT_METHOD_P2P:
                        _logger.info("MRIL_MOVEMENT_METHOD_P2P");

                        this->_RobotController.setMovementMethod(RobotController::MOVEMENT_METHODS::P2P);
                        break;

                    case MRIL_MOVEMENT_METHOD_LINEAR:
                        _logger.info("MRIL_MOVEMENT_METHOD_LINEAR");
                        this->_RobotController.setMovementMethod(RobotController::MOVEMENT_METHODS::LINEAR);
                        break;

                    case MRIL_MOVEMENT_METHOD_CIRCULAR:
                        _logger.warning("CIRCULAR not implemented");
                        break;
                    }
                }
                break;
            }

            case MRIL_COMMAND_VELOCITY:
            {
                if (commandType == CommandTypes::READ) {
                    responseBuffer += (String(MRIL_COMMAND_VELOCITY) + String(this->_RobotController.getMaxVelocity()));
                } else {
                    float value = atof(command + 1); //  V<val>

                    this->_RobotController.setMaxVelocity(value);
                    this->_AdditionalAxisController.setVelocity(value);
                }
                break;
            }

            case MRIL_COMMAND_SET_X:
            case MRIL_COMMAND_SET_Y:
            case MRIL_COMMAND_SET_Z:
            case MRIL_COMMAND_SET_A:
            case MRIL_COMMAND_SET_B:
            case MRIL_COMMAND_SET_C:
            {
                RobotController::POSITION pose;

                switch (symbol) {
                case MRIL_COMMAND_SET_X:
                    pose = RobotController::POSITION::X;
                    break;

                case MRIL_COMMAND_SET_Y:
                    pose = RobotController::POSITION::Y;
                    break;

                case MRIL_COMMAND_SET_Z:
                    pose = RobotController::POSITION::Z;
                    break;

                case MRIL_COMMAND_SET_A:
                    pose = RobotController::POSITION::A;
                    break;

                case MRIL_COMMAND_SET_B:
                    pose = RobotController::POSITION::B;
                    break;

                case MRIL_COMMAND_SET_C:
                    pose = RobotController::POSITION::C;
                    break;

                default:
                    break;

                    // should not get here lel
                }


                if (commandType == CommandTypes::READ) {
                    switch (symbol) {
                    case MRIL_COMMAND_SET_X:
                    case MRIL_COMMAND_SET_Y:
                    case MRIL_COMMAND_SET_Z:
                        responseBuffer += (String(symbol) + String(this->_RobotController.getCurrentPose(pose)));
                        break;

                    case MRIL_COMMAND_SET_A:
                    case MRIL_COMMAND_SET_B:
                    case MRIL_COMMAND_SET_C:
                        responseBuffer += (String(symbol) + String(this->_RobotController.getCurrentPose(pose) * RAD_TO_DEG));

                    default:
                        break;
                    }
                } else {
                    float value = atof(command + 1); // shift one because option V<opt><val>
                    _logger.info("setting " + String(symbol) + " to: " + String(value));

                    switch (symbol) {
                    case MRIL_COMMAND_SET_X:
                    case MRIL_COMMAND_SET_Y:
                    case MRIL_COMMAND_SET_Z:
                        this->_RobotController.setTargetPose(pose, value);
                        break;

                    case MRIL_COMMAND_SET_A:
                    case MRIL_COMMAND_SET_B:
                    case MRIL_COMMAND_SET_C:
                        this->_RobotController.setTargetPose(pose, value * DEG_TO_RAD);

                    default:
                        break;
                    }
                }
                break;
            }

            case MRIL_COMMAND_ROTATE:                                               // S M R4 1.12345 E
            {
                unsigned int option = (int)command[1] - 48;                         // shift one because option V<opt><val>

                if ((commandType == CommandTypes::READ) || (commandPointer == 2)) { // is still read, when one option is present
                    commandType = CommandTypes::READ;

                    if (option >= 6) {
                        responseBuffer += (String(MRIL_COMMAND_ROTATE) + String(option) +
                                                 String(this->_AdditionalAxisController.getCurrentAngle(option - 6) * RAD_TO_DEG));
                    } else {
                        responseBuffer += (String(MRIL_COMMAND_ROTATE) + String(option) +
                                                 String(this->_RobotController.getCurrentLogicalAngle(option) * RAD_TO_DEG));
                    }
                } else {
                    float value = atof(command + 2);

                    _logger.info("Rotate R" + String(option) + " value " + String(value));

                    // _logger.info(String(value));


                    if (option >= 6) {
                        this->_AdditionalAxisController.setAxisToAngle(option - 6, value / 180.0 * PI);
                    } else {
                        this->_RobotController.setTargetLogicalAngle(option, value * DEG_TO_RAD);
                    }

                }
                break;
            }

            case 'U':                                       // U1 00100 #wait for, U0 01020 #output
            {
                float value         = atof(command + 1);
                unsigned int option = (int)command[1] - 48; // shift one because option V<opt><val>

                if (option == 1) {
                    _logger.warning("not implemented");
                } else {
                    _logger.warning(value);
                }

                break;
            }

            case MRIL_COMMAND_LOGIC_INPUT:                 // S Q L I1 9 E
            {                                              // I<pin><state>
                unsigned int pin   = (int)command[1] - 48;
                unsigned int state = (int)command[2] - 48; // shift one because option I<opt><val>

                this->_IOLogic.addCondition(pin, state);
                break;
            }

            case MRIL_COMMAND_LOGIC_OUTPUT:                // S Q L O1 8 E     S Q L O0 8 E
            {                                              // O<state><pin>
                unsigned int pin   = (int)command[1] - 48;
                unsigned int state = (int)command[2] - 48; // shift one because option I<opt><val>

                this->_IOLogic.setOutput(pin, state);
                break;
            }

            case MRIL_COMMAND_WAIT:
            { // W<ms>
                unsigned long ms = atol(command + 1);
                this->_WaitController.waitMs(ms);
                break;
            }

            case MRIL_COMMAND_NUMBER:
            { // N0/1<number> 0 - executing - 1 executed
              // float value = atof(command+1);
                tmpCommandNumber = atoi(command + 1);
                this->_MRCPR.sendMessage("N0" + String(tmpCommandNumber));

                break;
            }

            case MRIL_COMMAND_TEST:

                break;

            case MRIL_COMMAND_HALT:
                break;

            default:
                _logger.warning("unknown Symbol ");
                _logger.warning((char)symbol);
            } // switch

            commandPointer = 0;
        }     // if command
    } // for

    this->_RobotController.endTransaction();

    if (commandType == CommandTypes::READ) {
        this->_MRCPR.sendMessage(responseBuffer);
        // read command - finished after parsing
        this->_MRCPR.sendMessage("N1" + String(tmpCommandNumber));
    } else if (commandType == CommandTypes::WRITE) {
        // write command - change the cN
        this->commandNumber = tmpCommandNumber;
    }


    _logger.info("Exiting consumeMRILFromBuffer");
}

void MRILParser::process() {
    if (this->_IOLogic.isDone() && !this->_RobotController.isMoving() &&  this->_WaitController.isDone()) { // todo add additionalAxis
        if (this->commandNumber > 0) {                                                                      // command number exists
            this->_MRCPR.sendMessage(String(MRIL_COMMAND_NUMBER) + "1" + String(this->commandNumber));
            this->commandNumber = -1;                                                                       // command was executed
        }
        this->done = true;
    } else {
        this->done = false;
    }
}

bool MRILParser::isDone() {
    return this->done;
}
