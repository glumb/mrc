#include "serialHelper.h"

stringCapture::stringCapture()
  : d() {
}

bool stringCapture::captureUInt16(uint16_t c) {
  d << c;
  return true;
}

bool stringCapture::captureUInt8(uint8_t c) {
  d << c;
  return true;
}

bool stringCapture::captureCStr(const uint8_t *buffer, size_t size) {
  d << std::string((const char*)buffer, size);
  return true;
}

void stringCapture::clear() {
  return d.str("");
}

std::string stringCapture::get() {
  return d.str();
}

