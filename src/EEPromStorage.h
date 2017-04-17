#ifndef EEPROMSTORAGE_H
#define EEPROMSTORAGE_H 1


#include <EEPROM.h>
#include "Logger.h"

#define EEPROM_SIZE 2048 // Teensy 3.2

namespace {
Logger log("EEPromStorage");

/**
 * EEPromStorage
 * |3|D|D|D|2|D|D|3|D|D|D|0|
 * |L|D|D|D|L|D|D|L|D|D|D|E|
 * L - length of message
 * D - data
 * E - end of messages
 */
class EEPromStorage {
public:

    /**
     * appends a message to EEProm storage
     * @param message array of chars
     * @param length  array length
     */
    void appendMessage(char message[], char length) {
        unsigned char pointer = 0;

        // find end of messages (could store this on init. saving memory this way)

        while (EEPROM.read(pointer) != 0){
          pointer += EEPROM.read(pointer) + 1;
        }

        if (pointer + length > EEPROM_SIZE) {
            log.error("access out of EEProm max size of: " + String(EEPROM_SIZE));
        }

        EEPROM.write(pointer, length);

        pointer++;

        for (size_t i = 0; i < length; i++) {
            EEPROM.write(pointer, message[i]);
            pointer++;
        }

        EEPROM.write(pointer, 0);
    }

    /**
     * returns the specified massage form EEProm, starting from 0..n
     * @param messageNumber message number 0..n
     * @param message       array of message bytes
     * @param length        length of returned message
     */
    char  getMessage(int messageNumber, char message[]) {
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

    /**
     * returns one byte from the EEProm
     * @param  address adress of byte 0..n
     * @return         byte in EEProm
     */
    char getByte(int address) {
        if (address > EEPROM_SIZE) {
            log.error("access out of EEProm max size of: " + String(EEPROM_SIZE));
        }

        return EEPROM.read(address);
    }

    void clear(){
      EEPROM.write(0,0);
    }
};
}

#endif
