#include "SPI.h"

static SPIMock* p_SPIMock = NULL;
SPIMock* SPIMockInstance() {
  if (!p_SPIMock) {
    p_SPIMock = new SPIMock();
  }
  return p_SPIMock;
}

void releaseSPIMock() {
  if (p_SPIMock) {
    delete p_SPIMock;
    p_SPIMock = NULL;
  }
}

void SPI_::begin() {
  p_SPIMock->begin();
}

void SPI_::usingInterrupt(uint8_t a) {
  p_SPIMock->usingInterrupt(a);
}

void SPI_::notUsingInterrupt(uint8_t a) {
  p_SPIMock->notUsingInterrupt(a);
}


void SPI_::beginTransaction(SPISettings a) {
  return p_SPIMock->beginTransaction(a);
}

uint8_t SPI_::transfer(uint8_t a) {
  return p_SPIMock->transfer(a);
}

uint16_t SPI_::transfer16(uint16_t a) {
  return p_SPIMock->transfer16(a);
}

void SPI_::transfer(void * a, size_t b) {
  return p_SPIMock->transfer(a, b);
}

void SPI_::endTransaction(void) {
  return p_SPIMock->endTransaction();
}

void SPI_::end(void) {
  return p_SPIMock->end();
}

void SPI_::setBitOrder(uint8_t a) {
  p_SPIMock->setBitOrder(a);
}

void SPI_::setDataMode(uint8_t a) {
  p_SPIMock->setDataMode(a);
}

void SPI_::setClockDivider(uint8_t a) {
  return p_SPIMock->setClockDivider(a);
}

void SPI_::attachInterrupt() {
  return p_SPIMock->attachInterrupt();
}

void SPI_::detachInterrupt() {
  return p_SPIMock->detachInterrupt();
}

// Preinstantiate Objects
SPI_ SPI;
