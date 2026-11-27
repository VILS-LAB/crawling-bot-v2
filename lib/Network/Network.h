#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>

#include "../../include/config.h"

// Forward declaration
class Display;

class Network {
public:
    Network(Display* display);
    void begin();
    void startOTATask();
    uint8_t getRobotNumber();
    const char* getSSID();
    const char* getHostname();
    const char* getIPAddress();

    // Stores a new robot number in EEPROM and rebuilds the SSID/hostname.
    // The access point itself keeps its old name until the next reboot.
    bool setRobotNumber(uint8_t number);

    static void otaTaskFunction(void* parameter);

private:
    Display* display;
    uint8_t robotNumber;
    char ssid[32];
    char otaHostname[32];
    TaskHandle_t otaTaskHandle;
    
    static const char* BASE_SSID;
    static const char* BASE_OTA_HOSTNAME;
    
    void setupAP();
    void setupOTA();
    uint8_t readRobotNumber();
    void saveRobotNumber(uint8_t number);
};

#endif // NETWORK_H
