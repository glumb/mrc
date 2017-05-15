#ifndef IOLOGIC_H
#define IOLOGIC_H 1

#define MAX_NUMBER_OF_PINS 10

class IOLogic {
public:

  IOLogic(unsigned int (&pinMap)[10]):pinMap(pinMap){};

  static const unsigned int IO_HIGH = 1;
  static const unsigned int IO_LOW = 0;
  static const unsigned int IO_UNDEFINED = 2;

  void addCondition(unsigned int pin,
                    unsigned int state);
  void setOutput(unsigned int pin,
                 unsigned int state);

  bool isDone();

  unsigned int getTargetState(unsigned int pin);

private:
  unsigned int conditionBuffer[MAX_NUMBER_OF_PINS][2];
  unsigned int conditionBufferLength = 0;
  // map physical pins to logic 0-9 pins
  unsigned int (&pinMap)[10];
};

#endif
