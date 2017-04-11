/**
 * Arduino Spark mock
 */
#ifndef SPARK_H
#define SPARK_H

#include <stdint.h>
#include <gmock/gmock.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class SparkMock {
  public:
    MOCK_METHOD2(variable, void(const char*, int*));
    MOCK_METHOD2(function, void(const char*, const char*));
    MOCK_METHOD2(publish, void(const char *, const char *));
    MOCK_METHOD2(subscribe, void(const char*, const char*));
    MOCK_METHOD0(connect, void());
    MOCK_METHOD0(disconnect, void());
    MOCK_METHOD0(connected, bool());
    MOCK_METHOD0(process, void());
    MOCK_METHOD0(deviceID, char * ());
    MOCK_METHOD0(sleep, void());
    MOCK_METHOD1(sleep, void(int));
    MOCK_METHOD2(sleep, void(const char*, int));
    MOCK_METHOD3(sleep, void(uint16_t, uint16_t, int));
    MOCK_METHOD0(syncTime, void());
};

class Spark_ {
  public:
    void variable(const char* name, int* p_value);
    void function(const char* funckey, const char* funcname);
    static void publish(const char *eventName, const char *data);
    void subscribe(const char* name, const char* cbHandler);
    void connect();
    void disconnect();
    bool connected(); // return true once connected
    void process(); // checks the Wi-Fi module for incoming message
    char* deviceID(); // return the device ID
    void sleep(); // put the module in sleep mode
    void sleep(int seconds); // put the module in sleep mode in [seconds] period
    void sleep(const char* sleep_mode,
               int seconds); // put the wifi module in deep sleep mode
    void sleep(uint16_t wakeUpPin, uint16_t edgeTriggerMode, int seconds);
    void syncTime(); // Synchronize time with the Spark Cloud
};
extern Spark_ Spark;

SparkMock* sparkMockInstance();
void releaseSparkMock();

#endif // SPARK_H
