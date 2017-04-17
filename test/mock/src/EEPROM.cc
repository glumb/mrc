/** Implementation of EEPROM mock **/

#include "EEPROM.h"

static EEPROMMock* p_EEPROMMock = NULL;
EEPROMMock* EEPROMMockInstance() {
  if (!p_EEPROMMock) {
    p_EEPROMMock = new EEPROMMock();
  }
  return p_EEPROMMock;
}

void releaseEEPROMMock() {
  assert (p_EEPROMMock != NULL);
  if (p_EEPROMMock) {
    delete p_EEPROMMock;
    p_EEPROMMock = NULL;
  }
}

uint8_t EEPROM_::read(int a) {
  assert (p_EEPROMMock != NULL);
  return p_EEPROMMock->read(a);
}

void EEPROM_::write(int a, uint8_t b) {
  assert (p_EEPROMMock != NULL);
  p_EEPROMMock->write(a, b);
}

// Preinstantiate Objects
EEPROM_ EEPROM;
