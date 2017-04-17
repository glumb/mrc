// Copyright 2014 http://switchdevice.com

#include "Spark.h"

static SparkMock* gSparkMock = NULL;
SparkMock* sparkMockInstance() {
  if(!gSparkMock) {
    gSparkMock = new SparkMock();
  }
  return gSparkMock;
}

void releaseSparkMock() {
  if(gSparkMock) {
    delete gSparkMock;
    gSparkMock = NULL;
  }
}

void Spark_::publish(const char *eventName, const char *data) {
  gSparkMock->publish(eventName, data);
}

void Spark_::variable(const char* name, int* p_value) {
  gSparkMock->variable(name, p_value);
}

void Spark_::function(const char* funckey, const char* funcname) {
  gSparkMock->function(funckey, funcname);
}

void Spark_::subscribe(const char* name, const char* cbHandler) {
  gSparkMock->subscribe(name, cbHandler);
}

void Spark_::connect() {
  gSparkMock->connect();
}

void Spark_::disconnect() {
  gSparkMock->disconnect();
}

bool Spark_::connected() {
  return gSparkMock->connected();
}

void Spark_::process() {
  gSparkMock->process();
}

char* Spark_::deviceID() {
  return gSparkMock->deviceID();
}

void Spark_::sleep() {
  gSparkMock->sleep();
}

void Spark_::sleep(int seconds) {
  gSparkMock->sleep(seconds);
}

void Spark_::sleep(const char* sleep_mode, int seconds) {
  gSparkMock->sleep(sleep_mode, seconds);
}

void Spark_::sleep(uint16_t wakeUpPin, uint16_t edgeTriggerMode, int seconds) {
  gSparkMock->sleep(wakeUpPin, edgeTriggerMode, seconds);
}

void Spark_::syncTime() {
  gSparkMock->syncTime();
}

// Preinstantiate Objects
Spark_ Spark;
