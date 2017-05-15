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
 *
 * todo we may want to do some caching here to prevent multiple reads for message count and position of last message.
 */
class EEPromStorage {
public:

    EEPromStorage() : messagePointer(1) {}

    /**
     * appends a message to EEProm storage
     * @param message array of chars
     * @param length  array length
     */
    void appendMessage(char message[],
                       char length);

    /**
     * returns the specified massage form EEProm, starting from 0..n
     * @param messageNumber message number 0..n
     * @param message       array of message bytes
     * @param length        length of returned message
     */
    char getMessage(unsigned int messageNumber,
                    char         message[]);

    /**
     * returns one byte from the EEProm
     * @param  address adress of byte 0..n
     * @return         byte in EEProm
     */
    char getByte(int address);

    /**
     * returns the number of messages in EEPROM
     * @return number of messages
     */
    int  getNumberOfMessages();

    int  getMessagePointer();

    char getNextMessage(char message[]);

    void clear();

private:

    int messagePointer;
};


#endif // ifndef EEPROMSTORAGE_H
