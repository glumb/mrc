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

MRILParser::MRILParser(RobotController          _RobotController,
                       IOLogic                   _IOLogic,
                       AdditionalAxisController *_AdditionalAxisController,
                       WaitController            _WaitController,
                       MRCPR                     _MRCPR) :
    _RobotController(_RobotController),
    _IOLogic(_IOLogic),
    _AdditionalAxisController(_AdditionalAxisController),
    _WaitController(_WaitController),
    _MRCPR(_MRCPR) {}

void MRILParser::parse(char mrilInstruction[], int length) {
    // cN changes when a non read Instruction is executed.
    int tmpCommandNumber = -1;

    enum CommandTypes { READ, WRITE, NONE };
    CommandTypes commandType     = CommandTypes::NONE; // 1:write, -1:read  you must not mix read and write commands
    CommandTypes lastCommandType = CommandTypes::NONE;

    char command[MRIL_COMMAND_SIZE];
    unsigned int commandPointer = 0;


    if (length == 0) { // buffer is not empty
        return;
    }


    // todo use atomic instead to also not invoke timers when setting axis R0 R1 and so on....
    this->_RobotController.startTransaction();

    for (size_t i = 0; i < length; i++) {
        char c = mrilInstruction[i];

        if ((c == '#') || (c == '(')) { // starting comment
            break;                      // do not interpret any other symbols for this instruction
        }


        // is a digit or dot or minus or uppercase letter
        if (((c >= 48) && (c <= 57)) || (c == 46) || ((c == 45) || ((c >= 65) && (c <= 90)))) {
            command[commandPointer] = c;
            commandPointer++;

            if (commandPointer >= MRIL_COMMAND_SIZE) {
                Serial.println("max option and value size exceeded");
            }
        } else  {
            // do not parse unknown chars
            continue;
            Serial.print("unknown char ");
            Serial.println((char)c);
        }

        _logger.info("ringBuffer parse char " + String((char)c));


        // command finished, interpret it.
        if (((c >= 65) && (c <= 90) && (i > 0)) || (i == length - 1)) { // is a character or end
            // no options means read command
            commandType = (commandPointer == 0) ? CommandTypes::READ : CommandTypes::WRITE;

            if ((commandType != lastCommandType) && (lastCommandType != CommandTypes::NONE)) {
                _logger.warning("Do not mix read and write instructions!");
            } else {
                lastCommandType = commandType;
            }

            char symbol = command[0];

            command[commandPointer] = '\0'; // NULL terminate to form a string

            commandPointer = 0;

            switch (symbol) {
            case MRIL_COMMAND_MOVEMENT_METHOD: {
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
                    Serial.println("CIRCULAR not implemented");
                    break;
                }
                break;
            }

            case MRIL_COMMAND_VELOCITY:
            {
                if (commandType == CommandTypes::READ) {
                    this->_MRCPR.sendMessage(String(MRIL_COMMAND_VELOCITY) + String(this->_RobotController.getMaxVelocity()));
                } else {
                    float value = atof(command + 1); //  V<val>

                    this->_RobotController.setMaxVelocity(value);
                    this->_AdditionalAxisController->setVelocity(value);
                }
                break;
            }

            case MRIL_COMMAND_SET_X:
            {
                if (commandType == CommandTypes::READ) {
                    this->_MRCPR.sendMessage(this->_RobotController.getCurrentPose(RobotController::POSITION::X));
                } else {
                    float value = atof(command + 1); // shift one because option V<opt><val>
                    _logger.info("setting x to: " + String(value));
                    this->_RobotController.setTargetPose(RobotController::POSITION::X, value);
                }
                break;
            }

            case MRIL_COMMAND_SET_Y:
            {
                float value = atof(command + 1); // shift one because option V<opt><val>
                _logger.info("setting y to: " + String(value));

                this->_RobotController.setTargetPose(RobotController::POSITION::Y, value);
                break;
            }

            case MRIL_COMMAND_SET_Z:
            {
                float value = atof(command + 1); // shift one because option V<opt><val>
                _logger.info("setting z to: " + String(value));

                this->_RobotController.setTargetPose(RobotController::POSITION::Z, value);
                break;
            }

            case MRIL_COMMAND_SET_A:
            {
                float value = atof(command + 1); // shift one because option V<opt><val>
                Serial.println(value);

                this->_RobotController.setTargetPose(RobotController::POSITION::A, value);
                break;
            }

            case MRIL_COMMAND_SET_B:
            {
                float value = atof(command + 1); // shift one because option V<opt><val>
                Serial.println(value);

                this->_RobotController.setTargetPose(RobotController::POSITION::B, value);
                break;
            }

            case MRIL_COMMAND_SET_C:
            {
                float value = atof(command + 1); // shift one because option V<opt><val>
                Serial.println(value);

                this->_RobotController.setTargetPose(RobotController::POSITION::C, value);
                break;
            }

            case MRIL_COMMAND_ROTATE:                       // S M R4 1.12345 E
            {
                float value         = atof(command + 1);
                unsigned int option = (int)command[1] - 48; // shift one because option V<opt><val>

                _logger.info("Rotate R" + String(option) + " value " + String(value));

                // _logger.info(String(value));

                this->_RobotController.setTargetAngle(option, value / 180.0 * PI);

                if (option >= 6) {
                    this->_AdditionalAxisController->setAxisToAngle(option - 6, value / 180.0 * PI);
                }

                // servos[option]->setTargetRadAngle(value / 180 * PI);

                // this->_RobotController.setAxisRotation(option,value);
                break;
            }

            case 'U':                                       // U1 00100 #wait for, U0 01020 #output
            {
                float value         = atof(command + 1);
                unsigned int option = (int)command[1] - 48; // shift one because option V<opt><val>

                if (option == 1) {
                    Serial.println("not implemented");
                } else {
                    Serial.println(value);
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
                Serial.println(ms);
                this->_WaitController.waitMs(ms);
                break;
            }

            case MRIL_COMMAND_NUMBER:
            { // N0/1<number> 0 - executing - 1 executed
              // float value = atof(command+1);
                tmpCommandNumber = atoi(command + 1);
                this->_MRCPR.sendMessage(String(MRIL_COMMAND_NUMBER) + "0" + String(this->commandNumber));

                break;
            }

            case MRIL_COMMAND_TEST:

                break;

            case MRIL_COMMAND_HALT:
                break;

            default:
                Serial.println("unknown Symbol ");
                Serial.println((char)symbol);
            } // switch
        }     // if command
    }         // for

    this->_RobotController.endTransaction();

    if (commandType == CommandTypes::READ) {
        // read command - finished after parsing
        this->_MRCPR.sendMessage(String(MRIL_COMMAND_NUMBER) + "1" + String(tmpCommandNumber));
    } else if (commandType == CommandTypes::WRITE) {
        // write command - change the cN
        this->commandNumber = tmpCommandNumber;
    }


    _logger.info("Exiting consumeMRILFromBuffer");
}

void MRILParser::process() {
    if (this->_IOLogic.isDone() && !this->_RobotController.isMoving() &&  this->_WaitController.isDone()) { // todo add additionalAxis
        if (this->commandNumber > 0) {                                                                       // command number exists
            this->_MRCPR.sendMessage(String(MRIL_COMMAND_NUMBER) + "1" + String(this->commandNumber));
            this->commandNumber = -1;                                                                        // command was executed
        }
        this->done = true;
    } else {
        this->done = false;
    }
}

bool MRILParser::isDone() {
    return this->done;
}
