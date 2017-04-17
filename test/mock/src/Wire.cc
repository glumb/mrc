#include "Wire.h"

static WireMock* p_WireMock = NULL;
WireMock* WireMockInstance() {
  if (!p_WireMock) {
    p_WireMock = new WireMock();
  }
  return p_WireMock;
}

void releaseWireMock() {
  if (p_WireMock) {
    delete p_WireMock;
    p_WireMock = NULL;
  }
}

void Wire_::begin() {
  p_WireMock->begin();
}

void Wire_::begin(uint8_t a) {
  p_WireMock->begin(a);
}

void Wire_::begin(int a) {
  p_WireMock->begin(a);
}

void Wire_::beginTransmission(uint8_t a) {
  p_WireMock->beginTransmission(a);
}


uint8_t Wire_::endTransmission(void) {
  return p_WireMock->endTransmission();
}

uint8_t Wire_::write(uint8_t a) {
  return p_WireMock->write(a);
}

uint8_t Wire_::write(char* a) {
  return p_WireMock->write(a);
}

uint8_t Wire_::write(uint8_t a, uint8_t b) {
  return p_WireMock->write(a, b);
}

uint8_t Wire_::available(void) {
  return p_WireMock->available();
}

uint8_t Wire_::read(void) {
  return p_WireMock->read();
}

void Wire_::onReceive(uint8_t* a) {
  p_WireMock->onReceive(a);
}

void Wire_::onRequest(uint8_t* a) {
  p_WireMock->onRequest(a);
}

uint8_t Wire_::endTransmission(uint8_t a) {
  return p_WireMock->endTransmission(a);
}

uint8_t Wire_::requestFrom(uint8_t a, uint8_t b) {
  return p_WireMock->requestFrom(a, b);
}

uint8_t Wire_::requestFrom(uint8_t a, uint8_t b, uint8_t c) {
  return p_WireMock->requestFrom(a, b, c);
}

// Preinstantiate Objects
Wire_ Wire;
