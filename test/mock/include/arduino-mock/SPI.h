// Header for SPI Mock

#ifndef __SPI_h__
#define __SPI_h__

#include <gmock/gmock.h>
#include <stdint.h>

class SPISettings {
    uint32_t _a;
    uint8_t _b;
    uint8_t _c;
  public:
    SPISettings(uint32_t a, uint8_t b, uint8_t c): _a(a), _b(b), _c(c) {}
    SPISettings();
    bool operator==(const SPISettings& rhs)const {
      return _a == rhs._a && _b == rhs._b && _c == rhs._c;
    }
};

class SPI_ {
  public:
    void begin();
    void usingInterrupt(uint8_t );
    void notUsingInterrupt(uint8_t );
    void beginTransaction(SPISettings );
    uint8_t transfer(uint8_t );
    uint16_t transfer16(uint16_t );
    void transfer(void *, size_t );
    void endTransaction(void);
    void end();
    void setBitOrder(uint8_t);
    void setDataMode(uint8_t );
    void setClockDivider(uint8_t clockDiv);
    void attachInterrupt();
    void detachInterrupt();
};

extern SPI_ SPI;

class SPIMock {
  public:
    MOCK_METHOD0(begin, void());
    MOCK_METHOD1(usingInterrupt, void(uint8_t));
    MOCK_METHOD1(notUsingInterrupt, void(uint8_t));
    MOCK_METHOD1(beginTransaction, void(SPISettings));
    MOCK_METHOD1(transfer, uint8_t(uint8_t));
    MOCK_METHOD1(transfer16, uint16_t(uint16_t));
    MOCK_METHOD2(transfer, void(void *, size_t));
    MOCK_METHOD0(endTransaction, void());
    MOCK_METHOD0(end, void());
    MOCK_METHOD1(setBitOrder, void(uint8_t));
    MOCK_METHOD1(setDataMode, void(uint8_t));
    MOCK_METHOD1(setClockDivider, void(uint8_t));
    MOCK_METHOD0(attachInterrupt, void());
    MOCK_METHOD0(detachInterrupt, void());
};

SPIMock* SPIMockInstance();
void releaseSPIMock();

#endif
