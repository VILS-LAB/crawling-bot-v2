#include <Arduino.h>
#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>
#include "../include/config.h"

namespace
{
Display display;
AHRS ahrs;
ServoControl servos(SERVO_PIN_DOWN, SERVO_PIN_UP);
Network network(&display);
Training training;
HealthCheck healthCheck(&display, &ahrs, &servos);

uint32_t lastTelemetryMs = 0;
uint32_t lastScreenMs = 0;
uint8_t screenPage = 0;

void showBootMessage(const char *line)
{
    display.clear();
    display.print("Crawling Bot V2", 0, 0);
    display.print(line, 0, 16);
    display.refresh();
}

void showTelemetry()
{
    const ImuTelemetry &imu = ahrs.getTelemetry();
    display.clear();

    if (screenPage == 0)
    {
        display.print("IMU orientation", 0, 0);
        display.setCursor(0, 16);
        display.print("R:");
        display.print((int)imu.roll);
        display.print(" P:");
        display.print((int)imu.pitch);
        display.print(" Y:");
        display.print((int)imu.yaw);
        display.setCursor(0, 32);
        display.print("T:");
        display.printFloat(imu.temperature, 1);
        display.print("C ");
        display.print(ahrs.isMoving() ? "MOV" : "STILL");
    }
    else if (screenPage == 1)
    {
        display.print("Linear accel (m/s2)", 0, 0);
        display.setCursor(0, 16);
        display.print("X:");
        display.printFloat(imu.linearAccel[0], 1);
        display.print(" Y:");
        display.printFloat(imu.linearAccel[1], 1);
        display.setCursor(0, 32);
        display.print("Z:");
        display.printFloat(imu.linearAccel[2], 1);
    }
    else
    {
        display.print("Velocity (m/s)", 0, 0);
        display.setCursor(0, 16);
        display.print("X:");
        display.printFloat(ahrs.getVelocityX(), 2);
        display.print(" Y:");
        display.printFloat(ahrs.getVelocityY(), 2);
        display.setCursor(0, 32);
        display.print("Z:");
        display.printFloat(ahrs.getVelocityZ(), 2);
        display.print(" S:");
        display.printFloat(ahrs.getSpeed(), 2);
    }

    display.setCursor(0, 52);
    display.print("Page ");
    display.print((int)(screenPage + 1));
    display.print("/3  CSV:115200");
    display.refresh();
    screenPage = (screenPage + 1) % 3;
}

void printTelemetryCsv()
{
    const ImuTelemetry &imu = ahrs.getTelemetry();
    Serial.print("IMU,");
    Serial.print(millis());
    Serial.print(',');
    Serial.print(imu.dt, 4);
    Serial.print(',');
    Serial.print(imu.accel[0], 4);
    Serial.print(',');
    Serial.print(imu.accel[1], 4);
    Serial.print(',');
    Serial.print(imu.accel[2], 4);
    Serial.print(',');
    Serial.print(imu.linearAccel[0], 4);
    Serial.print(',');
    Serial.print(imu.linearAccel[1], 4);
    Serial.print(',');
    Serial.print(imu.linearAccel[2], 4);
    Serial.print(',');
    Serial.print(imu.filteredLinearAccel[0], 4);
    Serial.print(',');
    Serial.print(imu.filteredLinearAccel[1], 4);
    Serial.print(',');
    Serial.print(imu.filteredLinearAccel[2], 4);
    Serial.print(',');
    Serial.print(imu.gyro[0], 3);
    Serial.print(',');
    Serial.print(imu.gyro[1], 3);
    Serial.print(',');
    Serial.print(imu.gyro[2], 3);
    Serial.print(',');
    Serial.print(imu.filteredGyro[0], 3);
    Serial.print(',');
    Serial.print(imu.filteredGyro[1], 3);
    Serial.print(',');
    Serial.print(imu.filteredGyro[2], 3);
    Serial.print(',');
    Serial.print(imu.roll, 2);
    Serial.print(',');
    Serial.print(imu.pitch, 2);
    Serial.print(',');
    Serial.print(imu.yaw, 2);
    Serial.print(',');
    Serial.print(ahrs.getVelocityX(), 4);
    Serial.print(',');
    Serial.print(ahrs.getVelocityY(), 4);
    Serial.print(',');
    Serial.print(ahrs.getVelocityZ(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionX(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionY(), 4);
    Serial.print(',');
    Serial.print(ahrs.getPositionZ(), 4);
    Serial.print(',');
    Serial.print(imu.temperature, 2);
    Serial.print(',');
    Serial.println(ahrs.isMoving() ? 1 : 0);
}
} // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(100);

    display.begin();
    showBootMessage("Starting...");

    network.begin();
    network.startOTATask();

    showBootMessage("Initializing IMU...");
    if (!ahrs.begin(network.getRobotNumber()))
    {
        showBootMessage("IMU FAILED");
        Serial.println("ERROR,IMU_INIT_FAILED");
        while (true)
            delay(1000);
    }

    showBootMessage("Keep still...");
    if (!ahrs.calibrate())
    {
        showBootMessage("CAL FAILED - PANEL");
        Serial.println("ERROR,IMU_CALIBRATION_FAILED,PANEL_AVAILABLE");
    }

#if ENABLE_SERVOS
    servos.begin();
#endif

    healthCheck.run();
    ahrs.serviceConsole();
    showBootMessage("Ready");
    lastTelemetryMs = millis();
    lastScreenMs = millis();
}

void loop()
{
    ahrs.update();
    ahrs.serviceConsole();
    const uint32_t now = millis();

    if (ahrs.isSerialTelemetryEnabled() &&
        now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryMs = now;
        printTelemetryCsv();
    }

    if (now - lastScreenMs >= DISPLAY_PAGE_INTERVAL_MS)
    {
        lastScreenMs = now;
        showTelemetry();
    }
}
