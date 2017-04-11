/**
 * Arduino IRremote mock
 * Used to mock out IRremote.h, an Arduino infrared remote library.
 * For info on IRremote, see Ken Shirriff's blog article:
 *   http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
 *   Note that Ken's latest code is now at https://github.com/shirriff/Arduino-IRremote
 */

#ifndef IRremote_h
#define IRremote_h

#include <stdint.h>
#include <gmock/gmock.h>

class decode_results {
  public:
    decode_results();
    int16_t decode_type; // NEC, SONY, RC5, UNKNOWN
    uint16_t panasonicAddress; // This is only used for decoding Panasonic data
    uint32_t value; // Decoded value
    int16_t bits; // Number of bits in decoded value
    volatile uint32_t *rawbuf; // Raw intervals in .5 us ticks
    int16_t rawlen; // Number of records in rawbuf.
};

class IRrecvMock {
  private:
    uint32_t irValue;

  public:
    IRrecvMock();

    void setIRValue(uint32_t value) {
      irValue = value;
    };
    uint32_t getIRValue() {
      return irValue;
    };

    MOCK_METHOD1(decode, int16_t (decode_results *));
    MOCK_METHOD0(enableIRIn, void ());
    MOCK_METHOD0(resume, void ());
};

class IRrecv_ {
  public:
    IRrecv_(int16_t recvpin);

    int16_t decode(decode_results *results);
    void enableIRIn();
    void resume();

    int16_t recvPin;
};

typedef IRrecv_ IRrecv;

IRrecvMock* irrecvMockInstance();
void releaseIRrecvMock();

#endif // IRremote_h
