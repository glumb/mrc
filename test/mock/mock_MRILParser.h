#ifndef MOCK_MRILPARSER_H
#define MOCK_MRILPARSER_H 1


#include "MRILParser.h"

class mock_MRILParser : public MRILParser {
public:

    mock_MRILParser(RobotController         & _RobotController,
                    IOLogic                 & _IOLogic,
                    AdditionalAxisController& _AdditionalAxisController,
                    WaitController          & _WaitController,
                    MRCPR                   & _MRCPR) :
        MRILParser(_RobotController,
                   _IOLogic,
                   _AdditionalAxisController,
                   _WaitController,
                   _MRCPR){};

    MOCK_METHOD2(parse, void(char mrilInstruction[], unsigned int  length));

private:
};

#endif // ifndef MOCK_MRILPARSER_H
