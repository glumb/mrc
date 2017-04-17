#ifndef EEPROMSTORAGE_H
#define EEPROMSTORAGE_H 1


#define EEPROM_SIZE 2048 // Teensy 3.2


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
    void appendMessage(char message[], char length);

    /**
     * returns the specified massage form EEProm, starting from 0..n
     * @param messageNumber message number 0..n
     * @param message       array of message bytes
     * @param length        length of returned message
     */
    char  getMessage(unsigned int messageNumber, char message[]);

    /**
     * returns one byte from the EEProm
     * @param  address adress of byte 0..n
     * @return         byte in EEProm
     */
    char getByte(int address);

    void clear();
};


#endif
