/**
EEPROM mock
**/

#ifndef __EEPROM_h__
#define __EEPROM_h__

#include <stdint.h>
#include <gmock/gmock.h>

class EEPROM_ {
  public:
    uint8_t read(int a);
    void write(int a,  uint8_t b);
};
extern EEPROM_ EEPROM;

class EEPROMMock {
  public:
    MOCK_METHOD1(read, uint8_t(int));
    MOCK_METHOD2(write, void(int, uint8_t));
};

EEPROMMock* EEPROMMockInstance();
void releaseEEPROMMock();
#endif
