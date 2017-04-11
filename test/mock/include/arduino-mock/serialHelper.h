/*
 * serialHelper - helper functions for unittesting serial functions.
 *
 */
#ifndef SERIALHELPER_H
#define SERIALHELPER_H

#include <stdint.h>
#include <string>
#include <sstream>

/*
 * stringCapture
 * This is a helper class which can be Invoked on EXPECT_CALL to gather data
 * from repeated calls to Serial functions. For example usage, see the unit
 * tests in test/serial_unittest.cc
 *
 * Example usage:
 *
 * EXPECT_CALL(*sm, write(Matcher<const uint8_t*>(_), (_)))
 *       .Times(AtLeast(1))
 *       .WillRepeatedly(Invoke(&c, &stringCapture::captureCStr));
 *
 */
class stringCapture {
  public:
    stringCapture();
    bool captureUInt8(uint8_t c);
    bool captureUInt16(uint16_t c);
    bool captureCStr(const uint8_t *buffer, size_t size);
    void clear();
    std::string get();

  private:
    std::stringstream d;
};


#endif // SERIALHELPER_H
