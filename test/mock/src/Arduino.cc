#include "arduino-mock/Arduino.h"

static ArduinoMock* arduinoMock = NULL;
ArduinoMock* arduinoMockInstance() {
  if(!arduinoMock) {
    arduinoMock = new ArduinoMock();
  }
  return arduinoMock;
}

void releaseArduinoMock() {
  if(arduinoMock) {
    delete arduinoMock;
    arduinoMock = NULL;
  }
}

ArduinoMock::ArduinoMock() {
  currentMillis = 0;
}

void pinMode(uint8_t a, uint8_t b) {
  assert (arduinoMock != NULL);
  arduinoMock->pinMode(a, b);
}
void digitalWrite(uint8_t a, uint8_t b) {
  assert (arduinoMock != NULL);
  arduinoMock->digitalWrite(a, b);
}

int digitalRead(uint8_t a) {
  assert (arduinoMock != NULL);
  return arduinoMock->digitalRead(a);
}

int analogRead(uint8_t a) {
  assert (arduinoMock != NULL);
  return arduinoMock->analogRead(a);
}

void analogReference(uint8_t mode) {
  UNUSED(mode);
}

void analogWrite(uint8_t a, int b) {
  assert (arduinoMock != NULL);
  arduinoMock->analogWrite(a, b);
}

unsigned long millis(void) {
  assert (arduinoMock != NULL);
  arduinoMock->millis();
  return arduinoMock->getMillis();
}

unsigned long micros(void) {
  return 0;
}
void delay(unsigned long a) {
  assert (arduinoMock != NULL);
  arduinoMock->delay(a);
}
void delayMicroseconds(unsigned int us) {
  UNUSED(us);
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
  UNUSED(pin);
  UNUSED(state);
  UNUSED(timeout);
  return 0;
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder,
              uint8_t val) {
  UNUSED(dataPin);
  UNUSED(clockPin);
  UNUSED(bitOrder);
  UNUSED(val);
}

uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  UNUSED(dataPin);
  UNUSED(clockPin);
  UNUSED(bitOrder);
  return 0;
}

void attachInterrupt(uint8_t, void (*)(void), int mode) {
  UNUSED(mode);
}

void detachInterrupt(uint8_t) {}
