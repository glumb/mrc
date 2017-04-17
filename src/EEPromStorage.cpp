#include "EEPromStorage.h"
#include <EEPROM.h>
#include "Logger.h"


static Logger logger("EEPromStorage");


void EEPromStorage::appendMessage(char message[], char length) {
    unsigned char pointer = 0;

    // find end of messages (could store this on init. saving memory this way)

    while (EEPROM.read(pointer) != 0) {
        pointer += EEPROM.read(pointer) + 1;
    }

    if (pointer + length > EEPROM_SIZE) {
        logger.error("access out of EEProm max size of: " + String(EEPROM_SIZE));
    }

    EEPROM.write(pointer, length);

    pointer++;

    for (size_t i = 0; i < length; i++) {
        EEPROM.write(pointer, message[i]);
        pointer++;
    }

    EEPROM.write(pointer, 0);
}

char EEPromStorage::getMessage(unsigned int messageNumber, char message[]) {
    unsigned char pointer = 0; // message length

    for (size_t i = 0; i < messageNumber; i++) {
        pointer += EEPROM.read(pointer) + 1;
    }

    char messageLength = EEPROM.read(pointer);

    pointer++;

    for (size_t i = 0; i < messageLength; i++) {
        message[i] = EEPROM.read(pointer + i);
    }

    return messageLength;
}

char EEPromStorage::getByte(int address) {
    if (address > EEPROM_SIZE) {
        logger.error("access out of EEProm max size of: " + String(EEPROM_SIZE));
    }

    return EEPROM.read(address);
}

void EEPromStorage::clear() {
    EEPROM.write(0, 0);
}
