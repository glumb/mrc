// Copyright 2014 http://switchdevice.com

#include "Serial.h"

static SerialMock* gSerialMock = NULL;
SerialMock* serialMockInstance() {
  if(!gSerialMock) {
    gSerialMock = new SerialMock();
  }
  return gSerialMock;
}

void releaseSerialMock() {
  if(gSerialMock) {
    delete gSerialMock;
    gSerialMock = NULL;
  }
}

// Serial.print(78, BIN) gives "1001110"
// Serial.print(78, OCT) gives "116"
// Serial.print(78, DEC) gives "78"
// Serial.print(78, HEX) gives "4E"
// Serial.println(1.23456, 0) gives "1"
// Serial.println(1.23456, 2) gives "1.23"
// Serial.println(1.23456, 4) gives "1.2346"

void printDouble(double num, int digits) {
  std::streamsize ss = std::cout.precision();
  std::cout << std::setprecision(digits) << std::fixed << num;
  std::cout.unsetf(std::ios::fixed);
  std::cout.precision (ss);
}

template<typename T> void printBase(T num, int base) {
  switch (base) {
  case BIN:
    assert (! "Need to implement this");
    break;
  case OCT:
    std::cout << std::oct;
    break;
  case DEC:
    std::cout << std::dec;
    break;
  case HEX:
    std::cout << std::hex;
    break;
  }
  std::cout << num << std::dec;
}

bool Serial_::printToCout = true;

void Serial_::setPrintToCout(bool flag) {
  printToCout = flag;
}

size_t Serial_::print(const char *s) {
  if (printToCout) {
    std::cout << s;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(s);
}

size_t Serial_::print(String s) {
  if (printToCout) {
    std::cout << s;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(s);
}
size_t Serial_::print(std::string s) {
  if (printToCout) {
    std::cout << s;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(s);
}

size_t Serial_::print(char c) {
  if (printToCout) {
    std::cout << c;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(c);
}

size_t Serial_::print(unsigned char c, int base) {
  if (printToCout) {
    printBase(c, base);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(c, base);
}

size_t Serial_::print(int num, int base) {
  if (printToCout) {
    printBase(num, base);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(num, base);
}

size_t Serial_::print(unsigned int num, int base) {
  if (printToCout) {
    printBase(num, base);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(num, base);
}

size_t Serial_::print(long num, int base) {
  if (printToCout) {
    printBase(num, base);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(num, base);
}

size_t Serial_::print(unsigned long num, int base) {
  if (printToCout) {
    printBase(num, base);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(num, base);
}

size_t Serial_::print(double num, int digits) {
  if (printToCout) {
    printDouble(num, digits);
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(num, digits);
}

size_t Serial_::println(const char *s) {
  if (printToCout) {
    std::cout << s << std::endl;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->println(s);
}

size_t Serial_::println(std::string s) {
  if (printToCout) {
    std::cout << s;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(s);
}

size_t Serial_::println(String s) {
  if (printToCout) {
    std::cout << s;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->print(s);
}


size_t Serial_::println(char c) {
  if (printToCout) {
    std::cout << c << std::endl;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->println(c);
}

size_t Serial_::println(unsigned char c, int base) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(c, base);
}

size_t Serial_::println(int num, int base) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(num, base);
}

size_t Serial_::println(unsigned int num, int base) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(num, base);
}

size_t Serial_::println(long num, int base) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(num, base);
}

size_t Serial_::println(unsigned long num, int base) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(num, base);
}

size_t Serial_::println(double num, int digits) {
  assert (gSerialMock != NULL);
  return gSerialMock->println(num, digits);
}

size_t Serial_::println(void) {
  if (printToCout) {
    std::cout << std::endl;
    return 0;
  }
  assert (gSerialMock != NULL);
  return gSerialMock->println();
}

size_t Serial_::write(uint8_t val) {
  assert (gSerialMock != NULL);
  return gSerialMock->write(val);
}

size_t Serial_::write(const char *str) {
  assert (gSerialMock != NULL);
  return gSerialMock->write(str);
}

size_t Serial_::write(const uint8_t *buffer, size_t size) {
  assert (gSerialMock != NULL);
  return gSerialMock->write(buffer, size);
}

uint8_t Serial_::begin(uint32_t port) {
  if (gSerialMock == NULL) {
    gSerialMock = serialMockInstance();
  }
  assert (gSerialMock != NULL);
  return gSerialMock->begin(port);
}

void Serial_::flush() {
  assert (gSerialMock != NULL);
  return gSerialMock->flush();
}

uint8_t Serial_::available() {
  assert (gSerialMock != NULL);
  return gSerialMock->available();
}

uint8_t Serial_::read() {
  assert (gSerialMock != NULL);
  return gSerialMock->read();
}

// Preinstantiate Objects
Serial_ Serial;
