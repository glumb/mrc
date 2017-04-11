#include "arduino-mock/IRremote.h"

// Taken from IRremoteInt.h
#define ERR 0
#define DECODED 1

static IRrecvMock* gIRrecvMock = NULL;
IRrecvMock* irrecvMockInstance() {
  if(!gIRrecvMock) {
    gIRrecvMock = new IRrecvMock();
  }
  return gIRrecvMock;
}

void releaseIRrecvMock() {
  if(gIRrecvMock) {
    delete gIRrecvMock;
    gIRrecvMock = NULL;
  }
}

IRrecvMock::IRrecvMock() {
  irValue = 0;
}

decode_results::decode_results() {
  decode_type = 0;
  panasonicAddress = 0;
  value = 0;
  bits = 0;
  rawbuf = NULL;
  rawlen = 0;
}

IRrecv_::IRrecv_(int16_t recvpin) {
  recvPin = recvpin;
}

int16_t IRrecv_::decode(decode_results *results) {
  assert (gIRrecvMock != NULL);
  gIRrecvMock->decode(results);
  assert (results != NULL);
  results->value = gIRrecvMock->getIRValue();
  return DECODED;
}

void IRrecv_::enableIRIn() {
  assert (gIRrecvMock != NULL);
  gIRrecvMock->enableIRIn();
}

void IRrecv_::resume() {
  assert (gIRrecvMock != NULL);
  gIRrecvMock->resume();
}

//// Preinstantiate Objects
//IRrecv_ IRrecv;
