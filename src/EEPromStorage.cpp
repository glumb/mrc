#include "EEPromStorage.h"
#include <EEPROM.h>
#include "Logger.h"


static Logger logger("EEPromStorage");


void EEPromStorage::appendMessage(char message[], char length) {
    unsigned int pointer = 0;

    // find end of messages (could store this on init. saving memory this way)

    while (EEPROM.read(pointer) != 0) {
        pointer += EEPROM.read(pointer) + 1;
    }

    if (pointer + length + 1  > EEPROM_SIZE) {
        logger.error("access out of EEProm max size of: " + String(EEPROM_SIZE)+" pointer "+String(pointer)+" length "+String(length));
        return;
    }

    EEPROM.write(pointer, length);

    pointer++;

    for (size_t i = 0; i < length; i++) {
        EEPROM.write(pointer, message[i]);
        pointer++;
    }

    EEPROM.write(pointer, 0);

    message[length] = 0;
    logger.info("writing message to EEPROM: " + String(message));
}

char EEPromStorage::getMessage(unsigned int messageNumber, char message[]) {
    unsigned int pointer = 0;           // message length

    for (size_t i = 0; i < messageNumber; i++) {
        if (EEPROM.read(pointer) == 0) { // last message
            return 0;
        }
        pointer += EEPROM.read(pointer) + 1;
    }

    char messageLength = EEPROM.read(pointer);

    logger.info("reading message number: " + String(messageNumber) + " at position "+String(pointer)+" length "+String(messageLength));

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

int EEPromStorage::getNumberOfMessages() {
    int messageCounter    = 0;
    unsigned int pointer = 0;

    while (EEPROM.read(pointer) != 0) {
        messageCounter++;
        pointer += EEPROM.read(pointer) + 1;
    }
    return messageCounter;
}

int EEPromStorage::getMessagePointer() {
    return this->messagePointer;
}

char EEPromStorage::getNextMessage(char message[]) {
    if (this->getNumberOfMessages() <= this->messagePointer) {
        this->messagePointer = 1;
    }
    logger.info("next message is " + String(this->messagePointer));
    return this->getMessage(this->messagePointer++, message);
}

void EEPromStorage::clear() {
    EEPROM.write(0, 0);
}
