#include "Network.h"
#include "../Display/Display.h"

const char* Network::BASE_SSID = "ESP32-AP-";
const char* Network::BASE_OTA_HOSTNAME = "ESP32-OTA-";

Network::Network(Display* display) : display(display), robotNumber(0), otaTaskHandle(NULL) {
}

void Network::begin() {
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    
    // Read or set robot number
    robotNumber = readRobotNumber();
    snprintf(ssid, sizeof(ssid), "%s%d", BASE_SSID, robotNumber);
    snprintf(otaHostname, sizeof(otaHostname), "%s%d", BASE_OTA_HOSTNAME, robotNumber);
    
    // Setup AP and OTA
    setupAP();
    setupOTA();
}

void Network::setupAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, AP_PASSWORD);
    Serial.println("AP Started");
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
}

void Network::setupOTA() {
    ArduinoOTA.setHostname(otaHostname);
    
    ArduinoOTA.onStart([this]() {
        Serial.println("OTA Start");
        if (display) {
            display->clear();
            display->print("OTA Update Start");
        }
    });
    
    ArduinoOTA.onEnd([this]() {
        Serial.println("\nOTA End");
        if (display) {
            display->clear();
            display->print("OTA Update Done");
        }
    });
    
    ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
        uint8_t percentage = total > 0 ? (progress * 100 / total) : 0;
        Serial.printf("Progress: %u%%\r", percentage);
        if (display) {
            display->drawProgressBar(percentage);
        }
    });
    
    ArduinoOTA.onError([this](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (display) {
            display->clear();
            display->print("OTA Error");
        }
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    Serial.print("OTA Hostname: ");
    Serial.println(otaHostname);
}

void Network::startOTATask() {
    xTaskCreatePinnedToCore(
        otaTaskFunction,
        "OTATask",
        4096,
        NULL,
        1,
        &otaTaskHandle,
        1
    );
}

void Network::otaTaskFunction(void* parameter) {
    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

uint8_t Network::readRobotNumber() {
    const uint8_t number = EEPROM.read(ROBOT_NUM_ADDR);
    if (number >= MIN_ROBOT_NUM && number <= MAX_ROBOT_NUM)
        return number;

    // Boot must never wait for a human: with an erased EEPROM (0xFF) or a
    // corrupt value the robot still has to come up, be reachable over OTA and
    // accept SETID. Default to robot 1 and say so loudly instead of blocking
    // on the serial port forever.
    Serial.println("WARN,ROBOT_NUM_INVALID,defaulting_to_1_use_SETID");
    Serial.print("WARN,ROBOT_NUM_RAW,");
    Serial.println(number);

    saveRobotNumber(MIN_ROBOT_NUM);
    return MIN_ROBOT_NUM;
}

void Network::saveRobotNumber(uint8_t number) {
    EEPROM.write(ROBOT_NUM_ADDR, number);
    EEPROM.commit();
    Serial.print("Saved robot number: ");
    Serial.println(number);
}

bool Network::setRobotNumber(uint8_t number) {
    if (number < MIN_ROBOT_NUM || number > MAX_ROBOT_NUM)
        return false;

    robotNumber = number;
    snprintf(ssid, sizeof(ssid), "%s%d", BASE_SSID, robotNumber);
    snprintf(otaHostname, sizeof(otaHostname), "%s%d", BASE_OTA_HOSTNAME, robotNumber);
    saveRobotNumber(robotNumber);
    return true;
}

uint8_t Network::getRobotNumber() {
    return robotNumber;
}

const char* Network::getSSID() {
    return ssid;
}

const char* Network::getHostname() {
    return otaHostname;
}

const char* Network::getIPAddress() {
    static char address[16];
    snprintf(address, sizeof(address), "%s", WiFi.softAPIP().toString().c_str());
    return address;
}
