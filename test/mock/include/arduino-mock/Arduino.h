/**
 * Arduino mock header
 */
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include "Serial.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#ifdef WIN32
#elif linux
#else
#define true 0x1
#define false 0x0
#endif

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

typedef uint8_t boolean;
typedef uint8_t byte;

void init(void);

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

#ifdef __cplusplus
} // extern "C"
#endif

#include <gmock/gmock.h>

#define UNUSED(expr) do { (void)(expr); } while (0)
#define F(x) (x)

class ArduinoMock {
  private:
    unsigned long  currentMillis;

  public:
    ArduinoMock();

    unsigned long getMillis() {
      return currentMillis;
    };

    void setMillisRaw (unsigned long milliseconds) {
      currentMillis = milliseconds;
    };
    void setMillisSecs(unsigned long seconds) {
      setMillisRaw(seconds *      1000);
    };
    void setMillisMins(unsigned long minutes) {
      setMillisRaw(minutes *   60 * 1000);
    };
    void setMillisHrs (float         hours)   {
      setMillisRaw(hours  * 60 * 60 * 1000);
    };

    void addMillisRaw (unsigned long milliseconds) {
      currentMillis += milliseconds;
    };
    void addMillisSecs(unsigned long seconds) {
      addMillisRaw(seconds *      1000);
    };
    void addMillisMins(unsigned long minutes) {
      addMillisRaw(minutes *   60 * 1000);
    };
    void addMillisHrs (float         hours)   {
      addMillisRaw(hours  * 60 * 60 * 1000);
    };

    MOCK_METHOD2(pinMode, void (uint8_t, uint8_t));
    MOCK_METHOD2(analogWrite, void (uint8_t, int));
    MOCK_METHOD2(digitalWrite, void (uint8_t, uint8_t));
    MOCK_METHOD1(digitalRead, int (int));
    MOCK_METHOD1(analogRead, int (int));
    MOCK_METHOD1(delay, void (int));
    MOCK_METHOD0(millis, unsigned long ());
};
ArduinoMock* arduinoMockInstance();
void releaseArduinoMock();

namespace {
std::string ReplaceAll(std::string str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}
}

class String : public std::string {
 public:
  String() {}
  String(const String& string) : String(string.c_str()) {}
  String(const std::string& string) : std::string(string) {}
  String(const char* string) : std::string(string) {}
  String(const float val) : std::string(std::to_string(val)) {}

  String substring(int start) {
    return substr(start);
  }
  String substring(int start, int end) {
    return substr(start, end-start);
  }

  String remove(int start) {
    return erase(start);
  }

  void replace(const String& from, const String& to) {
    ReplaceAll(*this, from, to);
  }

  int indexOf(const char needle) {
    return find(needle);
  }
};


#endif // ARDUINO_H
