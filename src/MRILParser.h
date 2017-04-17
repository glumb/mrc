#ifndef MRILPARSER_H
#define MRILPARSER_H 1


#include "IOLogic.h"
#include "RobotController.h"
#include "AdditionalAxisController.h"
#include "WaitController.h"
#include "CommunicationInterface.h"
#include "MRCPR.h"


#define MRIL_COMMAND_SIZE 20

#define INPUT_BUFFER_SIZE 80


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

class MRILParser {
public:

    MRILParser(RobotController         & _RobotController,
               IOLogic                 & _IOLogic,
               AdditionalAxisController& _AdditionalAxisController,
               WaitController          & _WaitController,
               MRCPR                   & _MRCPR);

    void parse(char mrilInstruction[],
               unsigned int  length);

    void process();

    bool isDone();

private:

    bool done = false;

    enum srcpMode { HALT, GO };
    srcpMode MRCPMODE = GO;

    long commandNumber = 0;

    RobotController& _RobotController;
    IOLogic& _IOLogic;
    AdditionalAxisController& _AdditionalAxisController;
    WaitController& _WaitController;
    MRCPR& _MRCPR;
};

#endif // ifndef MRILPARSER_H
