// Header for Wifi Mock

#ifndef __Wifi_h__
#define __Wifi_h__

#include <gmock/gmock.h>
#include <stdint.h>

class WiFi_ {
  public:
    void on();  // turns on WiFi module
    void off();  // turns off WiFi module
    void connect();  // Attempts to connect to the WiFi network
    void disconnect(); // Disconnect from the Wifi network
    bool connecting(); // Return true once the Core is attempting to connect
    bool ready();  // Return true once Core is connected
    void listen();  // Enter listening mode
    bool listening();  // Return true once listen() has been called
    void setCredentials(); // Allows user to set credentials
    bool clearCredentials(); // Clear all saved credentials
    bool hasCredentials();  // Return true if credentials have been already stored in CC3000's memory
    uint8_t macAddress(); // Return Mac Address of the device
    char* SSID();  // return SSID of the network
    int RSSI();  // return signal strength
    void ping(char*); // ping an IP address
    void ping(char*,
              uint8_t); // ping an IP address with a specified number of times
    char* localIP();  // Return local IP address
    char* subnetMask(); // Return Subnet mask of the network
    char* gatewayIP();  // Return the gateway IP address
};
extern WiFi_ WiFi;

class WiFiMock {
  public:
    MOCK_METHOD0(on, void());
    MOCK_METHOD0(off, void());
    MOCK_METHOD0(connect, void());
    MOCK_METHOD0(disconnect, void());
    MOCK_METHOD0(connecting, bool());
    MOCK_METHOD0(ready, bool());
    MOCK_METHOD0(listen, void());
    MOCK_METHOD0(listening, bool());
    MOCK_METHOD0(setCredentials, void());
    MOCK_METHOD0(clearCredentials, bool());
    MOCK_METHOD0(hasCredentials, bool());
    MOCK_METHOD0(macAddress, uint8_t());
    MOCK_METHOD0(SSID, char * ());
    MOCK_METHOD0(RSSI, int());
    MOCK_METHOD1(ping, void(char*));
    MOCK_METHOD2(ping, void(char*, uint8_t));
    MOCK_METHOD0(localIP, char * ());
    MOCK_METHOD0(subnetMask, char * ());
    MOCK_METHOD0(gatewayIP, char * ());
};

WiFiMock* WiFiMockInstance();
void releaseWiFiMock();

#endif
